"""Extract loaded DecalRoads without importing either planning library."""

import math
from collections.abc import Mapping


def extract_beamng_roads(beamng):
    """Return road records accepted by ``RoadBoundaryObstacleBuilder``.

    ``nodes`` contain world-space (x, y, z, full width) samples. ``left_edge``
    and ``right_edge`` retain the actual boundaries, including asymmetric
    roads and banking. All other simulator fields are kept in ``metadata``.
    Roads with non-positive drivability (including markings) are excluded.
    No widths, elevations, lane counts or connections are invented.
    """
    network = beamng.scenario.get_road_network(include_edges=True, drivable_only=True)
    if not isinstance(network, Mapping):
        raise ValueError("BeamNG returned an invalid road network")
    roads = []
    for road_id, data in network.items():
        try:
            drivability = float(data["drivability"])
            if not math.isfinite(drivability):
                raise ValueError("non-finite drivability")
            if drivability <= 0:
                continue
            nodes, left_edge, right_edge = [], [], []
            for edge in data["edges"]:
                points = [tuple(float(value) for value in edge[key])
                          for key in ("left", "middle", "right")]
                if any(len(point) != 3 or not all(map(math.isfinite, point))
                       for point in points):
                    raise ValueError("edges must contain finite XYZ coordinates")
                left, middle, right = points
                # Planning is in XY, so banked-road width must be projected too.
                width = math.hypot(left[0] - right[0], left[1] - right[1])
                if not math.isfinite(width):
                    raise ValueError("non-finite road width")
                nodes.append((*middle, width))
                left_edge.append(left)
                right_edge.append(right)
            if len(nodes) < 2 or not any(node[3] > 0 for node in nodes):
                continue
            if not any(a[:2] != b[:2] for a, b in zip(nodes, nodes[1:])):
                continue
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            raise ValueError(f"Invalid BeamNG geometry for road {road_id!r}: {exc}") from exc
        roads.append({
            "rid": str(road_id), "nodes": nodes,
            "left_edge": left_edge, "right_edge": right_edge,
            "metadata": {key: value for key, value in data.items() if key != "edges"},
        })
    if not roads:
        raise ValueError("BeamNG has no usable DecalRoads with positive drivability")
    return roads


def prepare_planning_roads(roads, goal_pos, endpoint_tolerance_m=0.0, goal_runout_m=0.0):
    """Apply explicit planning margins without changing the extracted records."""
    from shapely.geometry import LineString, Point

    for name, value in (("endpoint_tolerance_m", endpoint_tolerance_m),
                        ("goal_runout_m", goal_runout_m)):
        if not math.isfinite(value) or value < 0:
            raise ValueError(f"{name} must be finite and non-negative")
    result = list(roads)
    endpoints = [(index, node) for index, road in enumerate(roads)
                 for node in (road["nodes"][0], road["nodes"][-1])]
    if endpoint_tolerance_m:
        for index, (road_index, start) in enumerate(endpoints):
            for other_road_index, end in endpoints[index + 1:]:
                if (road_index != other_road_index
                        and math.dist(start[:2], end[:2]) > 1e-9
                        and math.dist(start[:3], end[:3]) <= endpoint_tolerance_m):
                    result.append({"rid": f"planning_join_{len(result)}", "nodes": [start, end]})
    if goal_runout_m:
        point = Point(goal_pos[:2])
        road, line = min(
            ((road, LineString([node[:2] for node in road["nodes"]])) for road in roads),
            key=lambda pair: pair[1].distance(point),
        )
        distance = line.project(point)
        for index, neighbour in ((0, 1), (-1, -2)):
            # BeamNG's float32 edges can differ slightly from the requested endpoint.
            if abs(distance - (0 if index == 0 else line.length)) > max(1e-3, endpoint_tolerance_m):
                continue
            endpoint, previous = road["nodes"][index], road["nodes"][neighbour]
            dx, dy = endpoint[0] - previous[0], endpoint[1] - previous[1]
            length = math.hypot(dx, dy)
            if length <= 1e-9:
                continue
            dx, dy = dx / length * goal_runout_m, dy / length * goal_runout_m
            extension = {"rid": "planning_goal_runout", "nodes": [
                endpoint, (endpoint[0] + dx, endpoint[1] + dy, *endpoint[2:]),
            ]}
            for key in ("left_edge", "right_edge"):
                if key in road:
                    edge = road[key][index]
                    extension[key] = [edge, (edge[0] + dx, edge[1] + dy, edge[2])]
            result.append(extension)
    return result
