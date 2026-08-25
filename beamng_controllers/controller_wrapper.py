from argparse import ArgumentParser
import random
import math

from beamngpy.sensors import Electrics

class ControllerWrapper:

    def next_control(self):
        """
        Override this method to implement your own control logic.
        This method is called by the controller process for each control iteration.
        :param electrics: The current state of the vehicle's electrics.
        :return: A tuple of (throttle, steering) values.
        """
        raise NotImplementedError("next_control must be implemented in a subclass.")


class StepController(ControllerWrapper):

    def __init__(self, vehicle):
        self.vehicle = vehicle
        vehicle.sensors.attach("electrics", Electrics())
        
    def next_control(self):
        self.vehicle.sensors.poll()
        electrics = self.vehicle.sensors["electrics"]
        throttle, steering, brake = self._compute_control_values(electrics)
        self.vehicle.control(throttle=throttle, steering=steering, brake=brake)

    def _compute_control_values(self, electrics):
        raise NotImplementedError("_compute_control_values must be implemented in a subclass.")


class RandomController(StepController):
    MIN_THROTTLE = 0.25
    MAX_THROTTLE = 0.75
    MAX_STEERING = 0.8

    def __init__(self, vehicle, min_throttle=MIN_THROTTLE, max_throttle=MAX_THROTTLE, max_steering=MAX_STEERING):
        super().__init__(vehicle)
        self.min_throttle = min_throttle
        self.max_throttle = max_throttle
        self.max_steering = max_steering
        self.rng = random.Random()
        self.index = 0

    def _compute_control_values(self, electrics):
        print(
            f"[CLIENT] sample {self.index + 1:02d}: "
            f"throttle={electrics["throttle_input"]!r}"
            f"steering={electrics["steering_input"]!r}"
            f"wheelspeed={electrics["wheelspeed"]!r}"
        )

        throttle = self.rng.uniform(self.min_throttle, self.max_throttle)

        # Blend randomness with a smooth oscillation so car visibly moves
        random_steer = self.rng.uniform(-self.max_steering, self.max_steering)
        wave_steer = math.sin(self.index * 0.5) * self.max_steering
        steering = (random_steer + wave_steer) / 2.0
        self.index += 1
        return throttle, steering, 0.0

    @staticmethod
    def build_parser(parser: ArgumentParser):
        parser.add_argument(
            "--min-throttle",
            type=float,
            default=RandomController.MIN_THROTTLE,
            help="Minimum throttle value (default: %(default)s)",
        )
        parser.add_argument(
            "--max-throttle",
            type=float,
            default=RandomController.MAX_THROTTLE,
            help="Maximum throttle value (default: %(default)s)",
        )
        parser.add_argument(
            "--max-steering",
            type=float,
            default=RandomController.MAX_STEERING,
            help="Maximum steering value (default: %(default)s)",
        )
