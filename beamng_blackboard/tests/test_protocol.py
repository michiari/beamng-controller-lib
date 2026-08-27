import pytest

from beamngpy import BeamNGpy
from beamng_blackboard import BeamNGBlackboard, Phase

def test_blackboard(running_beamng: BeamNGpy):
    """
    Test the basic "protocol" on a freshly started BeamNG.tech instance
    """
    
    blackboard = BeamNGBlackboard(running_beamng, poll_interval=0.01)

    # Ensure we use a fresh run_id to avoid messing up with past executions   
    # This is must be communicated to the client code along with the ip/port and scenario name
    
    # TODO Probably better to return a Snapshot or just return the id without all the fields
    run_id = blackboard.begin_run()

    snapshot = blackboard.read()

    assert snapshot is not None, "No snapshot"
    assert snapshot.run_id == run_id, f"{snapshot.run_id} is not the expected one {run_id}"
    assert snapshot.phase is Phase.PREPARING

    # Pretend we setup a scenario
    # Transition to ready
    blackboard.mark_ready(run_id)

    # Read the state 
    snapshot = blackboard.read()
    
    assert snapshot is not None, "No snapshot"
    assert snapshot.run_id == run_id, f"{snapshot.run_id} is not the expected one {run_id}"
    assert snapshot.phase is Phase.READY

    # TODO Skip the wait for READY check 

    # Start the scenario (Client Code)
    blackboard.mark_running(run_id)

    # Read the state 
    snapshot = blackboard.read()
    
    assert snapshot is not None, "No snapshot"
    assert snapshot.run_id == run_id, f"{snapshot.run_id} is not the expected one {run_id}"
    assert snapshot.phase is Phase.RUNNING

    # Check that also the given checking code is ok
    is_running = blackboard.is_running(run_id)

    assert is_running, "Scenario not running"

    blackboard.finish(run_id)
    # Read the state 
    snapshot = blackboard.read()
    
    assert snapshot is not None, "No snapshot"
    assert snapshot.run_id == run_id, f"{snapshot.run_id} is not the expected one {run_id}"
    assert snapshot.phase is Phase.ENDED