# beamng-blackboard

A lightweight blackboard-inspired component that uses BeamNG.tech as shared memory to synchronize remote driving agents with a central component managing the simulation of a scenario.

## Installation 

Check the compatibility of your environment here: [BeamNGpy Compatibility](https://documentation.beamng.com/api/beamngpy/master/compatibility.html)

```bash
python -m venv .venv
. .venv/bin/activate
python -m pip install -e ./requirements.txt
```

## Tests
We use `pytest` for implementing unit tests.

## Description

```
                     +----------+
new run ------------>| PREPARING |
                     +-----+----+
                           |
                           | main controller finished setup
                           v
                       +-------+
                       | READY |
                       +---+---+
                           |
                           | driver observes READY
                           v
                      +---------+
                      | RUNNING |
                      +----+----+
                           |
              +------------+-------------+
              |                          |
              v                          v
          +-------+                  +--------+
          | ENDED |                  | FAILED |
          +-------+                  +--------+
```

Each state is tagged with an unique run id, such that every update except creation includes the expected run_id.
Consequently, a delayed command from an old scenario cannot modify a new scenario.

```json
{
    "protocol": 1,
    "run_id": "997e9e8d...",
    "phase": "running",
    "revision": 4,
    "reason": None,
}
```
## Disclaimer
The code and descriptions have been generated using ChatGPT with the following prompt and then tested, fixed and adapted.

```
I need to develop a little python module that synchronizes two, possibly remote, independent processes. The specific scenario is the following:
\= A process starts and controls a driving simulation. BeamNG.tech is the reference simulator. The process uses the official beamngy Python APIs to interact with it. This process is in charge of setting up the scenario, monitoring the vehicles, and implementing the testing oracles. If an oracle triggers, the scenario must stop.
\= A second process, implementing a driving agent, connects to the running BeamNG.tech simulation, poll the vehicle sensors, computes the driving commands for driving the vehicle. This process also controls how simulation time flows (using the bemangpy step function)

The module I need to develop will use the running BeamNG simulator as shared memory to implement the IPC synchronization. To do so, it should rely on the primitive "queueLuaCommand" functionality that allows to run arbitrary LUA commands in the BeamNG simulator. Ideally, to set up a shared variable that each process could observe and change, making a blackboard system. I do not need strong primitive for synchronization, such as exclusive access and locks, but rather a lightweight implementation that ensures the following properties:

- The driver controller must wait until the scenario is ready, so it can start driving
- The driver controller stops as soon as the scenario is over (normal end, or oracle triggers a failure)
- The main controller can create new scenarios and past scenarios should not affect how elements synchronize in new scenarios
```