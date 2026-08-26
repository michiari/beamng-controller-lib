from dataclasses import dataclass

import time
import uuid
import json

from enum import Enum
from typing import Optional, Any

from beamngpy import BeamNGpy

# Definition of the states associated with the FSM modelling a scenario execution
class Phase(str, Enum):
    PREPARING = "preparing"
    READY = "ready"
    RUNNING = "running"
    ENDED = "ended"
    FAILED = "failed"

# Identify the final states
TERMINAL_PHASES = {Phase.ENDED.value, Phase.FAILED.value}


# Custom errors
class BlackboardError(RuntimeError):
    pass


class StaleRunError(BlackboardError):
    """The command referred to a run which is no longer current."""


@dataclass(frozen=True)
class Snapshot:
    run_id: str
    phase: str
    revision: int
    reason: Optional[str] = None

    @property
    def terminal(self) -> bool:
        return self.phase in TERMINAL_PHASES


# TODO Utility method, is this correct and/or needed?
def _lua_string(value: str) -> str:
    # TODO This does not seem to work...
    return value
    # """
    # Encode a Python string as a Lua long string.

    # This avoids having to deal with quotes, backslashes and UTF-8 escaping.
    # """
    # for n in range(10):
    #     eq = "=" * n
    #     closing = f"]{eq}]"

    #     if closing not in value:
    #         return f"[{eq}[{value}]{eq}]"

    # raise ValueError("Unable to encode string as Lua long string")

# TODO Utility method, is this correct and/or needed?
def _lua_optional_string(value: Optional[str]) -> str:
    return "nil" if value is None else _lua_string(value)


class BeamNGBlackboard:
    """
    Lightweight IPC blackboard stored inside BeamNG's Game Engine Lua VM to synchronize
    a central component controlling the simulator and creation of scenarios, and a client process
    implementing a driving agent that solves the scenarios.

    This component assumes that both python processes are connected to the same BeamNG.tech instance using BeamNGpy.

    This component assumes that both parties will call only and only the methods associated with them.
    """
    # TODO How could we enforce this behavior?

    # Define the namespace, i.e., the global lua variable to store the shared data
    _KEY = "__python_scenario_blackboard_v1"

    # TODO Clean up teh controller and remove * if not needed
    def __init__(
        self,
        bng: BeamNGpy,
        *,
        poll_interval: float = 0.02,
    ):
        
        self.bng = bng
        self.poll_interval = poll_interval

    # QUEUE_LUA_COMMAND force a tostring onto any object we return from BeamNG.tech, which for tables returns a plain reference. It does NOT return a StrDct but only a str.
    def _execute(self, lua: str) -> Any:    
        result_as_string = self.bng.control.queue_lua_command(
            lua,
            response=True,
        )
    
        if result_as_string is None:
            return None
        else:
            # Parse to dictionary/json
            return json.loads(result_as_string)

    # ------------------------------------------------------------------
    # Reading
    # ------------------------------------------------------------------

    def read(self) -> Optional[Snapshot]:
        """
        Read the current state of the simulation, if any
        """

        # Encode the parameter before sending it to BeamNG.tech
        key = _lua_string(self._KEY)

        # Pay attention to the call to jsonEncode!
        result = self._execute(
f"""
local b = rawget(_G, "{key}")

if b == nil then
    return nil
end

return jsonEncode({{
    run_id = b.run_id,
    phase = b.phase,
    revision = b.revision or 0,
    reason = b.reason
}})
"""
        )

        return Snapshot(
            run_id=result["run_id"],
            phase=Phase(result["phase"]),
            revision=int(result.get("revision", 0)),
            reason=result.get("reason"),
        )

    # ------------------------------------------------------------------
    # Main-controller operations
    # ------------------------------------------------------------------

    def begin_run(self, run_id: Optional[str] = None) -> str:
        """
        Create/reset the blackboard for a new scenario.

        This is intentionally unconditional. Only the main scenario
        controller should call this method.
        """

        # Random Unique ID
        run_id = run_id or uuid.uuid4().hex

        # Encode the parameters before sending them to BeamNG.tech
        key = _lua_string(self._KEY)
        rid = _lua_string(run_id)

        result = self._execute(
f"""
local b = {{
    protocol = 1,
    run_id = "{rid}",
    phase = "preparing",
    revision = 0,
    reason = nil
}}

rawset(_G, "{key}", b)

return jsonEncode({{
    run_id = b.run_id,
    phase = b.phase,
    revision = b.revision
}})
"""
)
        return result["run_id"]

    def mark_ready(self, run_id: str) -> None:
        """
        Transition the state of the scenario to be READY; this will notify the
        client that it can start controlling the vehicle.

        Note: This call succeeds only if the current state if PREPARING
        """
        self._transition(
            run_id,
            target=Phase.READY,
            allowed={Phase.PREPARING},
        )

    def finish(self, run_id: str, reason: Optional[str] = None) -> None:
        """
        Transition the state of the scenario to be END.

        Note: This call succeeds only if the current state if PREPARING|READY|RUNNING
        """
        self._transition(
            run_id,
            target=Phase.ENDED,
            allowed={
                Phase.PREPARING,
                Phase.READY,
                Phase.RUNNING,
            },
            reason=reason,
        )

    # TODO We can add a client-error transition if needed to notify that the 
    # client failed and the scenario should be stopped?
    def fail(self, run_id: str, reason: Optional[str] = None) -> None:
        """
        Transition the state of the scenario to be END with a FAIL.

        Note: This call succeeds only if the current state if PREPARING|READY|RUNNING
        """
        self._transition(
            run_id,
            target=Phase.FAILED,
            allowed={
                Phase.PREPARING,
                Phase.READY,
                Phase.RUNNING,
            },
            reason=reason,
        )

    # ------------------------------------------------------------------
    # Driver-controller operations
    # ------------------------------------------------------------------
    # TODO How the client knows what's the correct run_id?
    def mark_running(self, run_id: str) -> None:
        """
        Transition the state of the scenario to be RUNNING. Meaning the (client) driver agent
        takes control of the vehicle.

        Note: This call succeeds only if the current state if READY
        """
        self._transition(
            run_id,
            target=Phase.RUNNING,
            allowed={Phase.READY},
        )

    def wait_for_ready(
        self,
        *,
        after_run_id: Optional[str] = None,
        timeout: Optional[float] = None,
    ) -> Snapshot:
        """
        Wait for a READY scenario.

        If after_run_id is supplied, that run is ignored. This is what
        allows a persistent driver process to wait for the next scenario.
        """
        start = time.monotonic()

        while True:
            # Poll the state
            state = self.read()

            if (
                state is not None
                and state.phase == Phase.READY.value
                and (
                    after_run_id is None
                    or state.run_id != after_run_id
                )
            ):
                return state

            if timeout is not None:
                if time.monotonic() - start >= timeout:
                    raise TimeoutError(
                        "Timed out waiting for a ready BeamNG scenario"
                    )

            time.sleep(self.poll_interval)

    # TODO: where is this used?
    # THis should be withing the "wait_for_ready"
    # What shall we do if the client joins an already running scenario?
    def is_running(self, run_id: str) -> bool:
        state = self.read()

        return (
            state is not None
            and state.run_id == run_id
            and state.phase in {
                Phase.READY.value,
                Phase.RUNNING.value,
            }
        )

    # TODO Not sure if/why we need it.
    def wait_for_end(
        self,
        run_id: str,
        *,
        timeout: Optional[float] = None,
    ) -> Snapshot:
        start = time.monotonic()

        while True:
            state = self.read()

            # A different run means this run is definitely no longer active.
            if state is not None and state.run_id != run_id:
                return state

            if state is not None and state.terminal:
                return state

            if timeout is not None:
                if time.monotonic() - start >= timeout:
                    raise TimeoutError(
                        f"Timed out waiting for run {run_id} to finish"
                    )

            time.sleep(self.poll_interval)

    # ------------------------------------------------------------------
    # Internal transition primitive
    # ------------------------------------------------------------------

    def _transition(
        self,
        run_id: str,
        *,
        target: Phase,
        allowed: set[Phase],
        reason: Optional[str] = None,
    ) -> None:
        # Encodes the parameters
        key = _lua_string(self._KEY)
        rid = _lua_string(run_id)
        target_lua = _lua_string(target.value)
        reason_lua = _lua_optional_string(reason)

        # TODO Not sure about this... Is this checking whether we are allowed to transition?
        #   However, this is not done locally... but on the server...
        allowed_lua = ", ".join(
            f"[\"{_lua_string(p.value)}\"] = true"
            for p in allowed
        )

        # TODO: Why do we need a revision?
        result = self._execute(
f"""
local b = rawget(_G, "{key}")

if b == nil then
    return jsonEncode({{
        ok = false,
        error = "missing_blackboard"
    }})
end

if b.run_id ~= "{rid}" then
    return jsonEncode({{
        ok = false,
        error = "stale_run",
        current_run_id = b.run_id
    }})
end

local allowed = {{
    {allowed_lua}
}}

if not allowed[b.phase] then
    return jsonEncode({{
        ok = false,
        error = "invalid_transition",
        phase = b.phase
    }})
end

b.phase = "{target_lua}"
b.reason = {reason_lua if reason_lua == "nil" else '"' + reason_lua + '"'}
b.revision = (b.revision or 0) + 1

return jsonEncode({{
    ok = true,
    revision = b.revision
}})
"""
        )

        if result["ok"]:
            return

        error = result.get("error")

        if error == "stale_run":
            raise StaleRunError(
                f"Run {run_id!r} is no longer active; "
                f"current run is {result.get('current_run_id')!r}"
            )

        raise BlackboardError(
            f"Blackboard transition failed: {result}"
        )