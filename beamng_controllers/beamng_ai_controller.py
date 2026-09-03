from argparse import ArgumentParser

from .controller_wrapper import ControllerWrapper


DEFAULT_ROUTE_SPEED = None
DEFAULT_ROUTE_SPEED_MODE = None
DEFAULT_DRIVE_IN_LANE = True
DEFAULT_AVOID_CARS = True
ROUTE_SPEED_MODES = ("limit", "set", None)


def _parse_waypoints(value):
    waypoints = [waypoint.strip() for waypoint in value.split(",") if waypoint.strip()]
    if not waypoints:
        raise ValueError("Expected at least one waypoint")
    return waypoints


def _parse_wp_speeds(value):
    speeds = {}
    if not value:
        return speeds

    for item in value.split(","):
        item = item.strip()
        if not item:
            continue

        if "=" in item:
            waypoint, speed = item.split("=", 1)
        elif ":" in item:
            waypoint, speed = item.split(":", 1)
        else:
            raise ValueError(
                "Expected waypoint speed entries as '<waypoint>:<speed>' "
                "or '<waypoint>=<speed>'"
            )

        waypoint = waypoint.strip()
        if not waypoint:
            raise ValueError("Waypoint speed entry is missing a waypoint name")

        try:
            speeds[waypoint] = float(speed.strip())
        except ValueError as exc:
            raise ValueError(f"Invalid speed for waypoint {waypoint!r}") from exc

    return speeds


def _non_negative_float(value):
    number = float(value)
    if number < 0.0:
        raise ValueError("Expected a non-negative number")
    return number


def _positive_int(value):
    number = int(value)
    if number < 1:
        raise ValueError("Expected a positive integer")
    return number


def _aggression(value):
    number = float(value)
    if not 0.3 <= number <= 1.0:
        raise ValueError("Expected a value between 0.3 and 1.0")
    return number


class BeamNGAIController(ControllerWrapper):
    def __init__(
        self,
        vehicle,
        waypoints,
        wp_speeds=None,
        no_of_laps=None,
        route_speed=DEFAULT_ROUTE_SPEED,
        route_speed_mode=DEFAULT_ROUTE_SPEED_MODE,
        drive_in_lane=DEFAULT_DRIVE_IN_LANE,
        aggression=None,
        avoid_cars=DEFAULT_AVOID_CARS,
    ):
        self.vehicle = vehicle
        self.waypoints = waypoints
        if not waypoints:
            raise ValueError("waypoints must contain at least one waypoint")
        if no_of_laps is not None and no_of_laps < 1:
            raise ValueError("no_of_laps must be a positive integer")
        if route_speed is not None and route_speed < 0.0:
            raise ValueError("route_speed must be non-negative")
        if route_speed_mode is not None and route_speed_mode not in ROUTE_SPEED_MODES:
            modes = ", ".join(ROUTE_SPEED_MODES)
            raise ValueError(f"route_speed_mode must be one of: {modes}")
        if aggression is not None and not 0.3 <= aggression <= 1.0:
            raise ValueError("aggression must be between 0.3 and 1.0")
        self.drive_options = {
            "drive_in_lane": drive_in_lane,
            "avoid_cars": avoid_cars,
        }
        if wp_speeds is not None:
            self.drive_options["wp_speeds"] = wp_speeds
        if no_of_laps is not None:
            self.drive_options["no_of_laps"] = no_of_laps
        if route_speed is not None:
            self.drive_options["route_speed"] = route_speed
        if route_speed_mode is not None:
            self.drive_options["route_speed_mode"] = route_speed_mode
        if aggression is not None:
            self.drive_options["aggression"] = aggression
        self.started = False

    def next_control(self):
        if not self.started:
            self.started = True
            print("Starting AI driving using waypoints:", self.waypoints, self.drive_options)
            self.vehicle.ai.drive_using_waypoints(
                self.waypoints,
                **self.drive_options,
            )

    @staticmethod
    def build_parser(parser: ArgumentParser):
        parser.add_argument(
            "--waypoints",
            type=_parse_waypoints,
            required=True,
            help="Waypoints for the AI to follow as comma-separated list of waypoint names (e.g. 'wp1,wp2,wp3')",
        )
        parser.add_argument(
            "--wp-speeds",
            type=_parse_wp_speeds,
            default=None,
            help=(
                "Target speeds for individual waypoints as comma-separated "
                "entries (e.g. 'wp1:13.9,wp2:8.0')"
            ),
        )
        parser.add_argument(
            "--no-of-laps",
            type=_positive_int,
            default=None,
            help="Number of laps if the waypoint path is a loop",
        )
        parser.add_argument(
            "--route-speed",
            type=_non_negative_float,
            default=DEFAULT_ROUTE_SPEED,
            help="Route speed in m/s used with --route-speed-mode (default: %(default)s)",
        )
        parser.add_argument(
            "--route-speed-mode",
            choices=ROUTE_SPEED_MODES,
            default=DEFAULT_ROUTE_SPEED_MODE,
            help="How the AI applies --route-speed (default: %(default)s)",
        )
        parser.add_argument(
            "--drive-in-lane",
            dest="drive_in_lane",
            action="store_true",
            default=DEFAULT_DRIVE_IN_LANE,
            help="Keep the AI on the correct side of two-way roads",
        )
        parser.add_argument(
            "--no-drive-in-lane",
            dest="drive_in_lane",
            action="store_false",
            help="Allow the AI path finder to ignore lane direction",
        )
        parser.add_argument(
            "--aggression",
            type=_aggression,
            default=None,
            help="AI aggression from 0.3 to 1.0",
        )
        parser.add_argument(
            "--avoid-cars",
            dest="avoid_cars",
            action="store_true",
            default=DEFAULT_AVOID_CARS,
            help="Make the AI avoid other vehicles",
        )
        parser.add_argument(
            "--no-avoid-cars",
            dest="avoid_cars",
            action="store_false",
            help="Disable AI avoidance of other vehicles",
        )
