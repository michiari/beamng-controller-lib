# beamng-controllers

Shared controller interfaces and lightweight BeamNG.tech controllers. Install
this package in any controller environment that needs the shared interfaces:

```bash
python -m pip install -e ./beamng_controllers
```

This module also contains two example controllers:
- `RandomController`: simple controller that sets throttle and steering angles
  to random values at a given frequency.

- `BeamNGAIController`: controller that delegates driving to BeamNG AI for a list
  of BeamNG checkpoints supplied as a command line argument.

Both can be run by executing `beamng_controllers/vehicle_controller.py`.
Example usage:

```bash
python -m beamng_controllers.vehicle_controller --vehicle-id ego_vehicle random 
```

```bash
python -m beamng_controllers.vehicle_controller --vehicle-id ego_vehicle beamng-ai --waypoints final_destination
```
where `final_destination` has to be set in the scenario (e.g. with `scenario.add_checkpoints()`).

For more usage ooptions, consult
```bash
python -m beamng_controllers.vehicle_controller --help
python -m beamng_controllers.vehicle_controller random --help
python -m beamng_controllers.vehicle_controller beamng-ai --help
```
# Additions from 0.1.0
Initial delay option
