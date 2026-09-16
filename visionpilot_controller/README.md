# VisionPilot BeamNG Controller

A BeamNG.tech controller that uses
[VisionPilot](https://github.com/visionpilot-project/VisionPilot).

The VisionPilot repository currently needs some fixes before being used with this controller.
Please get it from this fork:
https://github.com/michiari/VisionPilot

Create a dedicated environment, install the shared packages, then install
VisionPilot and this controller:

```bash
python -m venv .venv
. .venv/bin/activate
python -m pip install -e ../beamng_controllers
python -m pip install -e ../beamng_blackboard
git clone https://github.com/michiari/VisionPilot ../VisionPilot
python -m pip install -e ../VisionPilot
python -m pip install -e .
```

For more usage options, consult
```bash
python -m visionpilot_controller.visionpilot_controller --help
```
