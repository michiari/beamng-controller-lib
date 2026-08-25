"""Convert BeamNG road geometry into obstacle walls for Safe-MPD.

The roads themselves are free space.  This module unions all road corridors and
turns the boundary of that union into the obstacle format used by
``mbd.envs.env.Env``:

* rectangles: ``[center_x, center_y, width, height, angle_radians]``

Unioning the corridors is important at junctions: it removes the boundaries of
overlapping road polygons, so the generated walls do not close an intersection.
"""

from __future__ import annotations

from argparse import ArgumentParser
from dataclasses import dataclass
import heapq
import math
from typing import Any, Iterable, Mapping, Sequence

from shapely import Geometry
from shapely.affinity import rotate, translate
from shapely.geometry import GeometryCollection, LineString, MultiLineString, Point
from shapely.geometry import MultiPolygon, Polygon, box
from shapely.ops import unary_union

from beamngpy.types import Float3

from mbd.envs import get_env
from mbd.envs.env import Env
from mbd.planners.mbd_planner import MBDConfig, run_diffusion, clear_jit_cache

from beamng_controllers.controller_wrapper import ControllerWrapper


_MIN_GEOMETRY_SIZE_M = 1e-9
_IMPORTED_CENTERLINE_CLEANUP_TOLERANCE_M = 1e-9
_IMPORTED_WIDTH_TOLERANCE_M = 1e-9
_NUMERICAL_HOLE_AREA_M2 = 1e-8
_DEFAULT_BUILDING_PROXIMITY_M = 1.0


SafeMPDRectangle = tuple[float, float, float, float, float]


@dataclass(frozen=True)
class RoadBoundaryObstacles:
    """Safe-MPD primitives and bounds containing roads and other obstacles."""

    rectangles: list[list[float]]
    road_bounds: tuple[float, float, float, float]


@dataclass(frozen=True)
class ImportedRoadGeometry:
    """Normalized road record returned by OpenDriveExtendedImporter."""

    rid: str
    nodes: tuple[tuple[float, float, float, float], ...]


class RoadBoundaryObstacleBuilder:
    """Build Safe-MPD road walls from one configured road corridor.

    An instance represents a single conversion job. It owns the normalized
    input geometries and wall configuration, computes the unioned road
    corridor once, and caches the resulting obstacle primitives. Keeping the
    corridor available is useful for callers that also need geometric checks
    or visualization without reconstructing the road map.

    ``imported_roads`` accepts the list returned by
    ``OpenDriveExtendedImporter.import_xodr``. Its rendered ``nodes`` and
    per-node widths define a variable-width corridor. Supplying
    ``road_width_m`` in this mode overrides every imported node width.

    ``building_polygons`` accepts either world-coordinate Shapely polygons or
    the complete ``(normalized_polygons, offsets)`` tuple returned by
    ``osm_roads.osm_buildings.extract_building_polygons``. Buildings within
    ``building_proximity_m`` of the road corridor become conservative
    rectangle decompositions aligned with their minimum-area orientation.
    """

    def __init__(
        self,
        road_centerlines: Iterable[LineString | MultiLineString] | None = None,
        xodr_roads: Iterable[Mapping[str, Any]] | None = None,
        *,
        building_polygons: Any = None,
        building_proximity_m: float = _DEFAULT_BUILDING_PROXIMITY_M,
        road_width_m: float | None = None,
        boundary_thickness_m: float = 0.25,
        simplify_tolerance_m: float = 0.0,
    ):
        supplied_source_count = sum(
            source is not None
            for source in (
                road_centerlines,
                xodr_roads,
            )
        )
        if supplied_source_count != 1:
            raise ValueError(
                "provide exactly one of road_centerlines, "
                "road_protection_polygons, or xodr_roads"
            )
        self.boundary_thickness_m = self._require_positive(
            "boundary_thickness_m", boundary_thickness_m
        )
        self.simplify_tolerance_m = self._require_non_negative(
            "simplify_tolerance_m", simplify_tolerance_m
        )
        self.building_proximity_m = self._require_non_negative(
            "building_proximity_m", building_proximity_m
        )
        self.building_polygons = self._normalize_building_polygons(
            building_polygons
        )

        if road_centerlines is not None:
            self.source_kind = "centerlines"
            self.road_centerlines = self._normalize_geometries(
                "road_centerlines",
                road_centerlines,
                (LineString, MultiLineString),
                "LineString or MultiLineString",
            )
            self.road_protection_polygons = None
            self.xodr_roads = None
            if road_width_m is None:
                raise ValueError("road_width_m is required with road_centerlines")
            self.road_width_m = self._require_positive(
                "road_width_m", road_width_m
            )
        else:
            self.source_kind = "imported_xodr_roads"
            self.road_protection_polygons = None
            self.road_width_m = (
                None
                if road_width_m is None
                else self._require_positive("road_width_m", road_width_m)
            )
            self.xodr_roads = self._normalize_imported_roads(xodr_roads)
            self.road_centerlines = tuple(
                LineString((node[0], node[1]) for node in road.nodes)
                for road in self.xodr_roads
            )

        self.corridor = self._build_corridor()
        self.nearby_building_polygons = tuple(
            polygon
            for polygon in self.building_polygons
            if polygon.distance(self.corridor) <= self.building_proximity_m
        )
        self.building_rectangles = [
            rectangle
            for polygon in self.nearby_building_polygons
            for rectangle in self._building_rectangles(polygon)
            if self._rect_dist_from_geom(rectangle, self.corridor) <= self.building_proximity_m
        ]
        self.boundary_rings = self._extract_boundary_rings()
        self._obstacles: RoadBoundaryObstacles | None = None

    @staticmethod
    def _require_positive(name: str, value: float, allow_infinity: bool = False) -> float:
        value = float(value)
        if value <= 0.0:
            raise ValueError(f"{name} must be a positive finite number")
        if not allow_infinity and not math.isfinite(value):
            raise ValueError(f"{name} must be a finite number")
        return value

    @staticmethod
    def _require_non_negative(name: str, value: float) -> float:
        value = float(value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be a non-negative finite number")
        return value

    @staticmethod
    def _normalize_geometries(
        name: str,
        geometries: Iterable[Any],
        expected_types: tuple[type, ...],
        expected_description: str,
    ) -> tuple[Any, ...]:
        try:
            items = tuple(geometries)
        except TypeError as exc:
            raise TypeError(
                f"{name} must be an iterable of Shapely geometries"
            ) from exc

        if any(not isinstance(geometry, expected_types) for geometry in items):
            raise TypeError(
                f"{name} may contain only {expected_description} geometries"
            )
        items = tuple(geometry for geometry in items if not geometry.is_empty)
        if not items:
            raise ValueError(f"{name} must contain at least one non-empty geometry")
        return items

    def _normalize_imported_roads(
        self,
        imported_roads: Iterable[Mapping[str, Any]],
    ) -> tuple[ImportedRoadGeometry, ...]:
        """Validate and freeze OpenDriveExtendedImporter return values."""

        try:
            roads = tuple(imported_roads)
        except TypeError as exc:
            raise TypeError(
                "imported_roads must be an iterable of road dictionaries"
            ) from exc

        normalized_roads = []
        for road_index, road in enumerate(roads):
            if not isinstance(road, Mapping):
                raise TypeError(
                    "imported_roads may contain only mapping-like road records"
                )
            if "nodes" not in road:
                raise ValueError(
                    f"imported road {road_index} does not contain a 'nodes' field"
                )
            try:
                nodes = tuple(road["nodes"])
            except TypeError as exc:
                raise TypeError(
                    f"nodes for imported road {road_index} must be iterable"
                ) from exc

            normalized_nodes = []
            for node_index, node in enumerate(nodes):
                try:
                    if len(node) < 4:
                        raise ValueError
                    x, y, z, imported_width = node[:4]
                    x, y, z = float(x), float(y), float(z)
                    width = (
                        self.road_width_m
                        if self.road_width_m is not None
                        else float(imported_width)
                    )
                except (TypeError, ValueError) as exc:
                    raise ValueError(
                        "each imported road node must contain numeric "
                        f"(x, y, z, width) values; invalid node {node_index} "
                        f"on road {road_index}"
                    ) from exc
                if not all(math.isfinite(value) for value in (x, y, z, width)):
                    raise ValueError(
                        f"node {node_index} on imported road {road_index} "
                        "contains a non-finite value"
                    )
                if width < 0.0:
                    raise ValueError(
                        f"node {node_index} on imported road {road_index} "
                        "has a negative width"
                    )
                normalized_nodes.append((x, y, z, width))

            if len(normalized_nodes) < 2:
                continue
            if not any(node[3] > 0.0 for node in normalized_nodes):
                continue
            normalized_roads.append(
                ImportedRoadGeometry(
                    rid=str(road.get("rid", f"road_{road_index}")),
                    nodes=tuple(normalized_nodes),
                )
            )

        if not normalized_roads:
            raise ValueError(
                "imported_roads contains no road with at least two nodes and "
                "a positive width"
            )
        return tuple(normalized_roads)

    @classmethod
    def _normalize_building_polygons(
        cls,
        building_data: Any,
    ) -> tuple[Polygon, ...]:
        """Normalize world polygons or ``extract_building_polygons`` output."""

        if building_data is None:
            return ()
        try:
            values = tuple(building_data)
        except TypeError as exc:
            raise TypeError(
                "building_polygons must be an iterable of polygons or the "
                "(polygons, offsets) output from extract_building_polygons"
            ) from exc

        extracted_output = (
            len(values) == 2
            and not isinstance(values[0], (Polygon, MultiPolygon))
            and not isinstance(values[1], (Polygon, MultiPolygon))
        )
        if extracted_output:
            try:
                polygons = tuple(values[0])
                offsets = tuple(values[1])
            except TypeError as exc:
                raise TypeError(
                    "building polygon and offset collections must be iterable"
                ) from exc
            if len(polygons) != len(offsets):
                raise ValueError(
                    "the number of building polygons and offsets must match"
                )
        else:
            polygons = values
            offsets = ((0.0, 0.0),) * len(polygons)

        normalized = []
        for index, (geometry, offset) in enumerate(zip(polygons, offsets)):
            if not isinstance(geometry, (Polygon, MultiPolygon)):
                raise TypeError(
                    "building_polygons may contain only Polygon or "
                    "MultiPolygon geometries"
                )
            try:
                offset_x, offset_y = float(offset[0]), float(offset[1])
            except (TypeError, ValueError, IndexError, KeyError) as exc:
                raise ValueError(
                    f"building offset {index} must contain numeric (x, y) values"
                ) from exc
            if not math.isfinite(offset_x) or not math.isfinite(offset_y):
                raise ValueError(
                    f"building offset {index} contains a non-finite value"
                )

            world_geometry = translate(
                geometry,
                xoff=offset_x,
                yoff=offset_y,
            )
            if not world_geometry.is_valid:
                world_geometry = world_geometry.buffer(0)
            normalized.extend(
                polygon
                for polygon in cls._polygon_parts(world_geometry)
                if not polygon.is_empty and polygon.area > _MIN_GEOMETRY_SIZE_M
            )
        return tuple(normalized)

    @classmethod
    def _building_rectangles(cls, polygon: Polygon) -> list[SafeMPDRectangle]:
        """Cover a polygon with tighter, minimum-rotation-aligned rectangles.

        The polygon is sliced at each vertex (and once between vertices) along
        both axes of its minimum rotated rectangle. Each slice component is
        conservatively covered by its local bounding box, and the decomposition
        with the smaller total rectangle area is returned. Consequently the
        result still covers the complete building while avoiding much of the
        empty area in a single minimum rotated rectangle, especially for
        concave and tapered footprints.
        """

        envelope = polygon.minimum_rotated_rectangle
        if not isinstance(envelope, Polygon) or envelope.is_empty:
            return []
        coordinates = list(envelope.exterior.coords)[:4]
        if len(coordinates) != 4:
            return []

        first, second, third = coordinates[:3]
        width = math.hypot(second[0] - first[0], second[1] - first[1])
        height = math.hypot(third[0] - second[0], third[1] - second[1])
        if width <= _MIN_GEOMETRY_SIZE_M or height <= _MIN_GEOMETRY_SIZE_M:
            return []

        angle = math.atan2(second[1] - first[1], second[0] - first[0])
        origin_x = float(polygon.centroid.x)
        origin_y = float(polygon.centroid.y)
        local_polygon = rotate(
            translate(polygon, xoff=-origin_x, yoff=-origin_y),
            -angle,
            origin=(0.0, 0.0),
            use_radians=True,
        )
        cosine, sine = math.cos(angle), math.sin(angle)

        rings = (local_polygon.exterior, *local_polygon.interiors)

        def distinct(values: Iterable[float]) -> list[float]:
            result: list[float] = []
            for value in sorted(float(item) for item in values):
                if not result or value - result[-1] > _MIN_GEOMETRY_SIZE_M:
                    result.append(value)
            return result

        def decompose(axis: int, *, refine: bool) -> list[SafeMPDRectangle]:
            vertex_cuts = distinct(
                coordinate[axis]
                for ring in rings
                for coordinate in ring.coords
            )
            cuts = vertex_cuts
            if refine:
                cuts = distinct(
                    [*vertex_cuts]
                    + [
                        (lower + upper) * 0.5
                        for lower, upper in zip(vertex_cuts, vertex_cuts[1:])
                    ]
                )
            local_min_x, local_min_y, local_max_x, local_max_y = (
                float(value) for value in local_polygon.bounds
            )
            rectangles: list[SafeMPDRectangle] = []
            for lower, upper in zip(cuts, cuts[1:]):
                if axis == 0:
                    slab = box(lower, local_min_y, upper, local_max_y)
                else:
                    slab = box(local_min_x, lower, local_max_x, upper)
                clipped = local_polygon.intersection(slab)
                for part in cls._polygon_parts(clipped):
                    min_x, min_y, max_x, max_y = (
                        float(value) for value in part.bounds
                    )
                    part_width = max_x - min_x
                    part_height = max_y - min_y
                    if (
                        part_width <= _MIN_GEOMETRY_SIZE_M
                        or part_height <= _MIN_GEOMETRY_SIZE_M
                    ):
                        continue
                    local_center_x = (min_x + max_x) * 0.5
                    local_center_y = (min_y + max_y) * 0.5
                    rectangles.append(
                        (
                            origin_x
                            + local_center_x * cosine
                            - local_center_y * sine,
                            origin_y
                            + local_center_x * sine
                            + local_center_y * cosine,
                            part_width,
                            part_height,
                            angle,
                        )
                    )
            return rectangles

        candidates = [
            rectangles
            for axis in (0, 1)
            for refine in (False, True)
            if (rectangles := decompose(axis, refine=refine))
        ]
        if not candidates:
            return []
        return min(
            candidates,
            key=lambda rectangles: (
                sum(rectangle[2] * rectangle[3] for rectangle in rectangles),
                len(rectangles),
            ),
        )

    @staticmethod
    def _rect_dist_from_geom(SafeMPDRectangle: SafeMPDRectangle, geometry: Geometry) -> float:
        """Return the distance from a rectangle to a Shapely geometry.

        The rectangle is represented as (center_x, center_y, width, height, angle).
        The polygon is a Shapely Polygon object.
        """
        center_x, center_y, width, height, angle = SafeMPDRectangle
        rectangle = box(-width / 2, -height / 2, width / 2, height / 2)
        rectangle = rotate(rectangle, angle, origin=(0.0, 0.0), use_radians=True)
        rectangle = translate(rectangle, xoff=center_x, yoff=center_y)
        return rectangle.distance(geometry)

    @classmethod
    def _polygon_parts(cls, geometry: Any) -> list[Polygon]:
        if isinstance(geometry, Polygon):
            return [geometry]
        if isinstance(geometry, MultiPolygon):
            return list(geometry.geoms)
        if isinstance(geometry, GeometryCollection):
            parts = []
            for child in geometry.geoms:
                parts.extend(cls._polygon_parts(child))
            return parts
        return []

    @classmethod
    def _line_parts(cls, geometry: Any) -> list[LineString]:
        if isinstance(geometry, LineString):
            return [geometry]
        if isinstance(geometry, MultiLineString):
            return list(geometry.geoms)
        if isinstance(geometry, GeometryCollection):
            parts = []
            for child in geometry.geoms:
                parts.extend(cls._line_parts(child))
            return parts
        return []

    def _build_corridor(self) -> Polygon | MultiPolygon:
        if self.source_kind == "centerlines":
            # Round caps and joins match run_simulation._buffer_polylines.
            corridor = unary_union(self.road_centerlines).buffer(
                self.road_width_m * 0.5,
                cap_style=1,
                join_style=1,
            )
        else:
            corridor = self._build_imported_road_corridor()

        # buffer(0) repairs common self-intersection defects without changing
        # valid polygons and discards non-polygonal geometry.
        if not corridor.is_valid:
            corridor = corridor.buffer(0)
        parts = [
            part for part in self._polygon_parts(corridor) if part.area > 0.0
        ]
        if not parts:
            raise ValueError("the supplied road geometries do not form a polygonal road")

        corridor = unary_union(parts)
        if self.simplify_tolerance_m:
            simplified = corridor.simplify(self.simplify_tolerance_m, preserve_topology=True)
            if not simplified.is_empty:
                corridor = simplified
        if not isinstance(corridor, (Polygon, MultiPolygon)):
            raise ValueError("the supplied road geometries do not form a polygonal road")
        return corridor

    def _build_imported_road_corridor(self) -> Polygon | MultiPolygon:
        """Build variable-width surfaces from imported XODR road nodes."""

        surface_parts = []
        for road in self.xodr_roads:
            widths = [node[3] for node in road.nodes]
            if max(widths) - min(widths) <= _IMPORTED_WIDTH_TOLERANCE_M:
                # The importer emits many nearly collinear samples even for a
                # straight XODR primitive. Buffering every sample separately
                # produces thousands of boundary edges and microscopic gaps.
                # A single buffered line is both simpler and more accurate for
                # a constant-width road.
                centerline = LineString(
                    (node[0], node[1]) for node in road.nodes
                ).simplify(
                    _IMPORTED_CENTERLINE_CLEANUP_TOLERANCE_M,
                    preserve_topology=False,
                )
                if centerline.length > _MIN_GEOMETRY_SIZE_M and widths[0] > 0.0:
                    surface_parts.append(
                        centerline.buffer(
                            widths[0] * 0.5,
                            cap_style=1,
                            join_style=1,
                        )
                    )
                continue

            for start, end in zip(road.nodes, road.nodes[1:]):
                x1, y1, _z1, width1 = start
                x2, y2, _z2, width2 = end
                dx, dy = x2 - x1, y2 - y1
                length = math.hypot(dx, dy)
                if length <= _MIN_GEOMETRY_SIZE_M:
                    continue

                normal_x, normal_y = -dy / length, dx / length
                half_width1, half_width2 = width1 * 0.5, width2 * 0.5
                segment_surface = Polygon(
                    [
                        (
                            x1 + normal_x * half_width1,
                            y1 + normal_y * half_width1,
                        ),
                        (
                            x2 + normal_x * half_width2,
                            y2 + normal_y * half_width2,
                        ),
                        (
                            x2 - normal_x * half_width2,
                            y2 - normal_y * half_width2,
                        ),
                        (
                            x1 - normal_x * half_width1,
                            y1 - normal_y * half_width1,
                        ),
                    ]
                )
                if not segment_surface.is_empty and segment_surface.area > 0.0:
                    surface_parts.append(segment_surface)

            # Node disks close segment joins and provide round road end caps.
            for x, y, _z, width in road.nodes:
                if width > 0.0:
                    surface_parts.append(Point(x, y).buffer(width * 0.5))

        if not surface_parts:
            raise ValueError("imported_roads do not form a positive-width road")
        corridor = unary_union(surface_parts)
        return self._remove_numerical_holes(corridor)

    @classmethod
    def _remove_numerical_holes(cls, geometry) -> Polygon | MultiPolygon:
        """Remove roundoff-sized holes without filling real map islands."""

        cleaned_polygons = []
        for polygon in cls._polygon_parts(geometry):
            real_holes = [
                interior.coords
                for interior in polygon.interiors
                if Polygon(interior).area >= _NUMERICAL_HOLE_AREA_M2
            ]
            cleaned_polygons.append(
                Polygon(
                    polygon.exterior.coords,
                    holes=real_holes,
                )
            )
        if not cleaned_polygons:
            raise ValueError("imported_roads do not form a polygonal road")
        return unary_union(cleaned_polygons)

    def _extract_boundary_rings(self) -> tuple[Sequence[tuple], ...]:
        rings: list[Sequence[tuple]] = []
        for polygon in self._polygon_parts(self.corridor):
            rings.append(tuple(polygon.exterior.coords))
            rings.extend(tuple(interior.coords) for interior in polygon.interiors)
        return tuple(rings)

    def road_heading_at(self, point: Point | Sequence[float]) -> float:
        """Return the local road heading at an ``(x, y)`` point, in radians.

        The closest centerline determines the road at the query point. Its
        coordinate order determines the heading direction, so the result can
        be passed directly to MBD as ``theta1``/``theta2``. At an intersection
        where several centerlines are equally close, input order breaks the
        tie deterministically.

        Polygon-only inputs cannot provide a heading because they contain no
        directional centerline information.
        """

        if self.road_centerlines is None:
            raise ValueError(
                "road headings require road_centerlines or xodr_roads; "
                "road_protection_polygons do not contain direction information"
            )

        query_point = self._normalize_query_point(point)
        if not self.corridor.covers(query_point):
            raise ValueError("point must lie within the road corridor")

        centerline_parts = [
            part
            for centerline in self.road_centerlines
            for part in self._line_parts(centerline)
            if part.length > _MIN_GEOMETRY_SIZE_M
        ]
        if not centerline_parts:
            raise ValueError("road centerlines contain no directional segment")

        centerline = min(
            enumerate(centerline_parts),
            key=lambda item: (item[1].distance(query_point), item[0]),
        )[1]
        distance = centerline.project(query_point)
        return self._heading_along_line(centerline, distance)

    def waypoints_between(
        self,
        start: Point | Sequence[float],
        goal: Point | Sequence[float],
        spacing_m: float,
    ) -> list[tuple[float, float, float]]:
        """Return route waypoints as ``(x, y, heading_radians)`` tuples.

        The route is the shortest connected path over the supplied road
        centerlines. Waypoints are separated by ``spacing_m`` measured along
        that route. The returned list includes the exact start and goal
        coordinates; its final interval can therefore be shorter than the
        requested spacing.

        Headings follow the direction of travel from ``start`` to ``goal``,
        rather than the stored direction of the source centerlines.
        """

        assert self.road_centerlines is not None
        spacing_m = self._require_positive("spacing_m", spacing_m, allow_infinity=True)
        start_point = self._normalize_query_point(start)
        goal_point = self._normalize_query_point(goal)
        if not self.corridor.covers(start_point):
            raise ValueError("start point must lie within the road corridor")
        if not self.corridor.covers(goal_point):
            raise ValueError("goal point must lie within the road corridor")

        noded_centerlines = unary_union(self.road_centerlines)
        network_segments = []
        for line in self._line_parts(noded_centerlines):
            coordinates = list(line.coords)
            for start_coordinate, end_coordinate in zip(
                coordinates, coordinates[1:]
            ):
                segment_start = (
                    float(start_coordinate[0]),
                    float(start_coordinate[1]),
                )
                segment_end = (
                    float(end_coordinate[0]),
                    float(end_coordinate[1]),
                )
                if math.dist(segment_start, segment_end) > _MIN_GEOMETRY_SIZE_M:
                    network_segments.append((segment_start, segment_end))
        if not network_segments:
            raise ValueError("road centerlines contain no routable segment")

        start_segment, start_parameter, start_projection = (
            self._nearest_network_projection(network_segments, start_point)
        )
        goal_segment, goal_parameter, goal_projection = (
            self._nearest_network_projection(network_segments, goal_point)
        )
        route_coordinates = self._shortest_network_route(
            network_segments,
            start_segment,
            start_parameter,
            start_projection,
            goal_segment,
            goal_parameter,
            goal_projection,
        )

        if len(route_coordinates) == 1:
            heading = self.road_heading_at(start_point)
            if start_point.equals(goal_point):
                return [(start_point.x, start_point.y, heading)]
            return [
                (start_point.x, start_point.y, heading),
                (goal_point.x, goal_point.y, heading),
            ]

        route = LineString(route_coordinates)
        sample_distances = [0.0]
        next_distance = spacing_m
        while next_distance < route.length - _MIN_GEOMETRY_SIZE_M:
            sample_distances.append(next_distance)
            next_distance += spacing_m
        sample_distances.append(route.length)

        waypoints = []
        for index, distance in enumerate(sample_distances):
            route_point = route.interpolate(distance)
            if index == 0:
                x, y = start_point.x, start_point.y
            elif index == len(sample_distances) - 1:
                x, y = goal_point.x, goal_point.y
            else:
                x, y = route_point.x, route_point.y
            waypoints.append(
                (x, y, self._heading_along_line(route, distance))
            )
        return waypoints

    @classmethod
    def _heading_along_line(
        cls,
        centerline: LineString,
        distance: float,
    ) -> float:
        """Return the travel-direction tangent at a distance along a line."""

        sample_distance = min(0.5, max(0.01, centerline.length * 0.01))
        before = centerline.interpolate(max(0.0, distance - sample_distance))
        after = centerline.interpolate(
            min(centerline.length, distance + sample_distance)
        )
        dx, dy = after.x - before.x, after.y - before.y
        if math.hypot(dx, dy) <= _MIN_GEOMETRY_SIZE_M:
            query_point = centerline.interpolate(distance)
            return cls._nearest_segment_heading(centerline, query_point)
        return math.atan2(dy, dx)

    @staticmethod
    def _nearest_network_projection(
        segments: Sequence[
            tuple[tuple[float, float], tuple[float, float]]
        ],
        point: Point,
    ) -> tuple[int, float, tuple[float, float]]:
        candidates = []
        for index, (start, end) in enumerate(segments):
            dx, dy = end[0] - start[0], end[1] - start[1]
            length_squared = dx * dx + dy * dy
            parameter = (
                (point.x - start[0]) * dx + (point.y - start[1]) * dy
            ) / length_squared
            parameter = min(1.0, max(0.0, parameter))
            projection = (
                start[0] + parameter * dx,
                start[1] + parameter * dy,
            )
            candidates.append(
                (
                    math.hypot(
                        projection[0] - point.x,
                        projection[1] - point.y,
                    ),
                    index,
                    parameter,
                    projection,
                )
            )
        _distance, index, parameter, projection = min(candidates)
        return index, parameter, projection

    @classmethod
    def _shortest_network_route(
        cls,
        segments: Sequence[
            tuple[tuple[float, float], tuple[float, float]]
        ],
        start_segment: int,
        start_parameter: float,
        start_projection: tuple[float, float],
        goal_segment: int,
        goal_parameter: float,
        goal_projection: tuple[float, float],
    ) -> list[tuple[float, float]]:
        """Split the projected edges and run Dijkstra over the road network."""

        cut_parameters = {index: [0.0, 1.0] for index in range(len(segments))}
        cut_parameters[start_segment].append(start_parameter)
        cut_parameters[goal_segment].append(goal_parameter)

        adjacency: dict[
            tuple[float, float],
            list[tuple[tuple[float, float], float]],
        ] = {}
        positions: dict[tuple[float, float], tuple[float, float]] = {}

        for index, (segment_start, segment_end) in enumerate(segments):
            dx = segment_end[0] - segment_start[0]
            dy = segment_end[1] - segment_start[1]
            parameters = []
            for parameter in sorted(cut_parameters[index]):
                if not parameters or abs(parameter - parameters[-1]) > 1e-12:
                    parameters.append(parameter)
            segment_points = [
                (
                    segment_start[0] + parameter * dx,
                    segment_start[1] + parameter * dy,
                )
                for parameter in parameters
            ]
            for point_a, point_b in zip(segment_points, segment_points[1:]):
                key_a = cls._network_node_key(point_a)
                key_b = cls._network_node_key(point_b)
                edge_length = math.dist(point_a, point_b)
                if edge_length <= _MIN_GEOMETRY_SIZE_M or key_a == key_b:
                    continue
                positions.setdefault(key_a, point_a)
                positions.setdefault(key_b, point_b)
                adjacency.setdefault(key_a, []).append((key_b, edge_length))
                adjacency.setdefault(key_b, []).append((key_a, edge_length))

        start_key = cls._network_node_key(start_projection)
        goal_key = cls._network_node_key(goal_projection)
        if start_key == goal_key:
            return [positions.get(start_key, start_projection)]

        distances = {start_key: 0.0}
        previous: dict[tuple[float, float], tuple[float, float]] = {}
        queue = [(0.0, start_key)]
        while queue:
            distance, node = heapq.heappop(queue)
            if distance > distances.get(node, math.inf):
                continue
            if node == goal_key:
                break
            for neighbor, edge_length in adjacency.get(node, ()):
                candidate_distance = distance + edge_length
                if candidate_distance < distances.get(neighbor, math.inf):
                    distances[neighbor] = candidate_distance
                    previous[neighbor] = node
                    heapq.heappush(queue, (candidate_distance, neighbor))

        if goal_key not in distances:
            raise ValueError("start and goal do not lie on the same connected road network")

        path = [goal_key]
        while path[-1] != start_key:
            path.append(previous[path[-1]])
        path.reverse()
        return [positions[node] for node in path]

    @staticmethod
    def _network_node_key(
        point: tuple[float, float],
    ) -> tuple[float, float]:
        """Merge coordinates that differ only by sub-nanometre roundoff."""

        return round(point[0], 9), round(point[1], 9)

    @staticmethod
    def _normalize_query_point(point: Point | Sequence[float]) -> Point:
        if isinstance(point, Point):
            if point.is_empty:
                raise ValueError("point must not be empty")
            x, y = point.x, point.y
        else:
            try:
                x, y = point[0], point[1]
            except (TypeError, IndexError, KeyError) as exc:
                raise TypeError(
                    "point must be a Shapely Point or an (x, y) coordinate sequence"
                ) from exc

        try:
            x, y = float(x), float(y)
        except (TypeError, ValueError) as exc:
            raise ValueError("point coordinates must be numeric") from exc
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("point coordinates must be finite")
        return Point(x, y)

    @staticmethod
    def _nearest_segment_heading(centerline: LineString, point: Point) -> float:
        """Handle degenerate local samples by using the closest valid segment."""

        coordinates = list(centerline.coords)
        segments = []
        for index, (start, end) in enumerate(
            zip(coordinates, coordinates[1:])
        ):
            dx, dy = end[0] - start[0], end[1] - start[1]
            if math.hypot(dx, dy) > _MIN_GEOMETRY_SIZE_M:
                segments.append(
                    (LineString((start, end)).distance(point), index, dx, dy)
                )
        if not segments:
            raise ValueError("road centerline contains no directional segment")
        _distance, _index, dx, dy = min(segments)
        return math.atan2(dy, dx)

    def _rectangles_for_ring(
        self,
        coordinates: Sequence[tuple],
    ) -> list[list[float]]:
        rectangles = []
        for start, end in zip(coordinates, coordinates[1:]):
            x1, y1 = float(start[0]), float(start[1])
            x2, y2 = float(end[0]), float(end[1])
            dx, dy = x2 - x1, y2 - y1
            length = math.hypot(dx, dy)
            if length <= _MIN_GEOMETRY_SIZE_M:
                continue
            # Overlap adjacent wall ends so rectangle-only boundaries do not
            # leave collision gaps at corners.
            length += self.boundary_thickness_m
            rectangles.append(
                [
                    (x1 + x2) * 0.5,
                    (y1 + y2) * 0.5,
                    length,
                    self.boundary_thickness_m,
                    math.atan2(dy, dx),
                ]
            )
        return rectangles

    def build(self) -> RoadBoundaryObstacles:
        """Generate and cache road-boundary and building obstacles."""

        if self._obstacles is None:
            rectangles: list[list[float]] = []
            for ring in self.boundary_rings:
                rectangles.extend(self._rectangles_for_ring(ring))

            # Buildings are solid footprints, independent of the primitive
            # type selected for the road boundary.
            rectangles.extend(
                list(rectangle) for rectangle in self.building_rectangles
            )

            min_x, min_y, max_x, max_y = (
                float(value) for value in self.corridor.bounds
            )
            for center_x, center_y, width, height, angle in (
                self.building_rectangles
            ):
                cosine, sine = abs(math.cos(angle)), abs(math.sin(angle))
                x_extent = cosine * width * 0.5 + sine * height * 0.5
                y_extent = sine * width * 0.5 + cosine * height * 0.5
                min_x = min(min_x, center_x - x_extent)
                min_y = min(min_y, center_y - y_extent)
                max_x = max(max_x, center_x + x_extent)
                max_y = max(max_y, center_y + y_extent)

            self._obstacles = RoadBoundaryObstacles(
                rectangles=rectangles,
                road_bounds=(min_x, min_y, max_x, max_y),
            )
        return self._obstacles

    def plot_debug(
        self,
        waypoints: Iterable[Sequence[float]],
        *,
        show: bool = True,
        ax=None,
    ):
        """Plot the generated obstacles and ``(x, y, heading)`` waypoints.

        Rectangle patches correspond exactly to the primitives returned by
        :meth:`build`. Heading arrows are drawn in radians. The
        returned ``(figure, axes)`` pair can be customized or saved by the
        caller; pass ``show=False`` to avoid opening a window.
        """

        from matplotlib import pyplot as plt
        from matplotlib.patches import Polygon as PolygonPatch

        try:
            waypoint_values = tuple(waypoints)
        except TypeError as exc:
            raise TypeError(
                "waypoints must be an iterable of (x, y, heading) values"
            ) from exc
        if not waypoint_values:
            raise ValueError("waypoints must contain at least one point")

        normalized_waypoints = []
        for index, waypoint in enumerate(waypoint_values):
            try:
                if len(waypoint) < 3:
                    raise ValueError
                x, y, heading = map(float, waypoint[:3])
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"waypoint {index} must contain numeric (x, y, heading) values"
                ) from exc
            if not all(math.isfinite(value) for value in (x, y, heading)):
                raise ValueError(f"waypoint {index} contains a non-finite value")
            normalized_waypoints.append((x, y, heading))

        obstacles = self.build()
        created_axes = ax is None
        if created_axes:
            figure, ax = plt.subplots(figsize=(10.0, 8.0))
        else:
            figure = ax.figure

        corridor_label = "Road corridor"
        for polygon in self._polygon_parts(self.corridor):
            x_coordinates, y_coordinates = polygon.exterior.xy
            ax.plot(
                x_coordinates,
                y_coordinates,
                color="0.4",
                linewidth=1.0,
                linestyle="--",
                label=corridor_label,
                zorder=1,
            )
            corridor_label = None
            for interior in polygon.interiors:
                x_coordinates, y_coordinates = interior.xy
                ax.plot(
                    x_coordinates,
                    y_coordinates,
                    color="0.4",
                    linewidth=1.0,
                    linestyle="--",
                    zorder=1,
                )

        centerline_label = "Road centerline"
        if self.road_centerlines is not None:
            for centerline in self.road_centerlines:
                for line in self._line_parts(centerline):
                    x_coordinates, y_coordinates = line.xy
                    ax.plot(
                        x_coordinates,
                        y_coordinates,
                        color="0.55",
                        linewidth=0.8,
                        label=centerline_label,
                        zorder=1,
                    )
                    centerline_label = None

        road_rectangle_count = (
            len(obstacles.rectangles) - len(self.building_rectangles)
        )
        road_rectangle_label = "Road rectangle obstacles"
        building_rectangle_label = "Building obstacles"
        for rectangle_index, (
            center_x,
            center_y,
            width,
            height,
            angle,
        ) in enumerate(obstacles.rectangles):
            is_building = rectangle_index >= road_rectangle_count
            cosine, sine = math.cos(angle), math.sin(angle)
            half_width, half_height = width * 0.5, height * 0.5
            vertices = [
                (
                    center_x + local_x * cosine - local_y * sine,
                    center_y + local_x * sine + local_y * cosine,
                )
                for local_x, local_y in (
                    (-half_width, -half_height),
                    (half_width, -half_height),
                    (half_width, half_height),
                    (-half_width, half_height),
                )
            ]
            ax.add_patch(
                PolygonPatch(
                    vertices,
                    closed=True,
                    facecolor="#9467bd" if is_building else "#d62728",
                    edgecolor="#5e3c76" if is_building else "#8c1b1b",
                    linewidth=0.5,
                    alpha=0.55,
                    label=(
                        building_rectangle_label
                        if is_building
                        else road_rectangle_label
                    ),
                    zorder=2,
                )
            )
            if is_building:
                building_rectangle_label = None
            else:
                road_rectangle_label = None

        waypoint_x = [waypoint[0] for waypoint in normalized_waypoints]
        waypoint_y = [waypoint[1] for waypoint in normalized_waypoints]
        waypoint_heading = [waypoint[2] for waypoint in normalized_waypoints]
        ax.plot(
            waypoint_x,
            waypoint_y,
            color="#1f77b4",
            marker="o",
            markersize=4.0,
            linewidth=1.5,
            label="Waypoint route",
            zorder=5,
        )

        road_min_x, road_min_y, road_max_x, road_max_y = obstacles.road_bounds
        map_diagonal = math.hypot(
            road_max_x - road_min_x,
            road_max_y - road_min_y,
        )
        arrow_length = max(
            self.boundary_thickness_m * 3.0,
            map_diagonal * 0.03,
        )
        ax.quiver(
            waypoint_x,
            waypoint_y,
            [arrow_length * math.cos(heading) for heading in waypoint_heading],
            [arrow_length * math.sin(heading) for heading in waypoint_heading],
            angles="xy",
            scale_units="xy",
            scale=1.0,
            color="#1f77b4",
            width=0.004,
            zorder=6,
        )
        for index, (x, y, _heading) in enumerate(normalized_waypoints):
            ax.annotate(
                str(index),
                (x, y),
                xytext=(4, 4),
                textcoords="offset points",
                fontsize=8,
                color="#1f4f7a",
                zorder=7,
            )

        plot_min_x = min(road_min_x, *waypoint_x)
        plot_min_y = min(road_min_y, *waypoint_y)
        plot_max_x = max(road_max_x, *waypoint_x)
        plot_max_y = max(road_max_y, *waypoint_y)
        plot_margin = max(
            1.0,
            map_diagonal * 0.05,
            self.boundary_thickness_m,
        )
        ax.set_xlim(plot_min_x - plot_margin, plot_max_x + plot_margin)
        ax.set_ylim(plot_min_y - plot_margin, plot_max_y + plot_margin)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_title(
            "Road boundary obstacles and waypoints "
            f"({len(obstacles.rectangles)} rectangles, "
            f"{len(self.building_rectangles)} building rectangles)"
        )
        ax.grid(True, alpha=0.25)
        handles, labels = ax.get_legend_handles_labels()
        unique_labels = dict(zip(labels, handles))
        ax.legend(unique_labels.values(), unique_labels.keys())
        if created_axes:
            figure.tight_layout()
        if show:
            plt.show()
        return figure, ax

    def create_mbd_env(
        self,
        *,
        environment_margin_m: float = 1.0,
        resolution: float = 0.1
    ):
        """Create a navigation ``mbd.Env`` for this configured road map."""

        resolution = self._require_positive("resolution", resolution)
        obstacles = self.build()

        min_x, min_y, max_x, max_y = obstacles.road_bounds
        wall_padding = environment_margin_m + self.boundary_thickness_m * 0.5
        x_range = (min_x - wall_padding, max_x + wall_padding)
        y_range = (min_y - wall_padding, max_y + wall_padding)
        env = Env(
            width=x_range[1] - x_range[0],
            height=y_range[1] - y_range[0],
            case="navigation",
            resolution=resolution,
        )
        env.set_rectangle_obs(obstacles.rectangles, coordinate_mode="center", padding=0.0)
        env.set_plot_limits(x_range, y_range)
        return env


class SafeMPDController(ControllerWrapper):
    """A controller that uses Safe-MPD to generate a navigation environment."""

    def __init__(
            self,
            vehicle,
            xodr_roads,
            lane_width: float,
            spawn_pos: Float3,
            goal_pos: Float3,
            waypoint_spacing_m: float = 25.0,
            time_step: float = 0.25,
            max_speed: float = 10.0,
            trailer: bool = False,
            seed: int = 42,
            dynamics: str = "tt2d",
            samples: int = 1000,  # number of samples
            horizon: int = 500,  # horizon
            diffusion_steps: int = 100, # number of diffusion steps
            buildings: Any = None,
            debug: bool = False
        ):
        self.vehicle = vehicle
        self.trajectory_states = None
        self.ai_script = None

        obstacle_builder = RoadBoundaryObstacleBuilder(
            xodr_roads=xodr_roads,
            road_width_m=lane_width * 2.0,
            building_polygons=buildings
        )
        self.base_env_config = obstacle_builder.create_mbd_env()

        self.config = MBDConfig(
            seed=seed,
            env_name=dynamics,
            case="navigation",
            Nsample=samples,
            Hsample=horizon,
            Ndiffuse=diffusion_steps,
            dt=time_step,
            render=debug,
            show_animation=False,
            save_animation=False,
            save_denoising_animation=False,
            verbose=debug,
            num_trailers=1 if trailer else 0,
            #l2=1.0, # trailer length
            # lh=1.0, # hitch length
            #trailer_width=0.5,
            v_max=max_speed
        )

        obstacles = obstacle_builder.build()
        print(f"Road centerlines: {len(obstacle_builder.road_centerlines)}; width: {lane_width * 2.0:.2f} m")
        print(
            "Boundary obstacles: "
            f"{len(obstacles.rectangles)} rectangles"
        )

        self.waypoints = obstacle_builder.waypoints_between(spawn_pos, goal_pos, spacing_m=waypoint_spacing_m)
        print(f"Generated waypoints: {len(self.waypoints)} points, {self.waypoints[0]} -> {self.waypoints[-1]}")
        self.first_waypoint, self.last_waypoint = self.waypoints[0], self.waypoints[-1]

        if debug:
            obstacle_builder.plot_debug(self.waypoints, show=True)


    def _create_robot_env(self, init_pos: Float3, goal_pos: Float3):
        """Create the MBD robot environment around the configured base Env."""
        self.robot_env = get_env(
            self.config.env_name,
            case=self.config.case,
            env_config=self.base_env_config,
            dt=self.config.dt,
            H=self.config.Hsample,
            motion_preference=self.config.motion_preference,
            collision_penalty=self.config.collision_penalty,
            enable_shielded_rollout_collision=self.config.enable_shielded_rollout_collision,
            hitch_penalty=self.config.hitch_penalty,
            enable_shielded_rollout_hitch=self.config.enable_shielded_rollout_hitch,
            enable_projection=self.config.enable_projection,
            enable_guidance=self.config.enable_guidance,
            reward_threshold=self.config.reward_threshold,
            ref_reward_threshold=self.config.ref_reward_threshold,
            max_w_theta=self.config.max_w_theta,
            hitch_angle_weight=self.config.hitch_angle_weight,
            l1=self.config.l1,
            l2=self.config.l2,
            lh=self.config.lh,
            lf1=self.config.lf1,
            lr=self.config.lr,
            lf2=self.config.lf2,
            lr2=self.config.lr2,
            tractor_width=self.config.tractor_width,
            trailer_width=self.config.trailer_width,
            v_max=self.config.v_max,
            delta_max_deg=self.config.delta_max_deg,
            a_max=self.config.a_max,
            omega_max=self.config.omega_max,
            d_thr_factor=self.config.d_thr_factor,
            k_switch=self.config.k_switch,
            steering_weight=self.config.steering_weight,
            preference_penalty_weight=self.config.preference_penalty_weight,
            heading_reward_weight=self.config.heading_reward_weight,
            terminal_reward_threshold=self.config.terminal_reward_threshold,
            terminal_reward_weight=self.config.terminal_reward_weight,
            ref_pos_weight=self.config.ref_pos_weight,
            ref_theta1_weight=self.config.ref_theta1_weight,
            ref_theta2_weight=self.config.ref_theta2_weight,
            num_trailers=self.config.num_trailers,
        )
        self.robot_env.set_init_pos(
            x=init_pos[0],
            y=init_pos[1],
            theta1=init_pos[2],
            theta2=init_pos[2],
        )
        self.robot_env.set_goal_pos(
            x=goal_pos[0],
            y=goal_pos[1],
            theta1=goal_pos[2],
            theta2=goal_pos[2],
        )


    def set_vehicle(self, vehicle):
        """Set the vehicle to be controlled by this Safe-MPD controller."""
        self.vehicle = vehicle


    def get_true_spawn_pos(self) -> Float3:
        return self.first_waypoint
    

    def get_true_goal_pos(self) -> Float3:
        return self.last_waypoint


    def _set_next_init_goal_pos(self):
        if len(self.waypoints) < 2:
            raise ValueError("Not enough waypoints to set next init and goal positions.")
        next_init = self.waypoints.pop(0)
        next_goal = self.waypoints[0]
        self._create_robot_env(init_pos=next_init, goal_pos=next_goal)
        print(f"Set next init position: {next_init}, next goal position: {next_goal}")
        clear_jit_cache()


    def next_control(self):
        if self.vehicle is None:
            raise ValueError("Vehicle must be set before calling next_control.")

        self._set_next_init_goal_pos()
        self._plan()
        self.vehicle.ai.set_script(self._trajectory_to_ai_script())


    def control_entire_route(self):
        if self.vehicle is None:
            raise ValueError("Vehicle must be set before calling control_entire_route.")

        if self.ai_script is None:
            self.plan_entire_route()
        self.vehicle.ai.set_script(self.ai_script)


    def _plan(self):
        print("Running Safe-MPD planner...")
        rew_final, _actions, self.trajectory_states, timing_info = run_diffusion(
            args=self.config,
            env=self.robot_env,
        )
        print("MPD planning completed.")
        print(f"Number of trajectory states: {len(self.trajectory_states)}")
        print(f"Final reward: {float(rew_final):.3f}")
        print(f"Pure diffusion time: {timing_info['pure_diffusion_time']:.2f}s")
        print(f"Total time: {timing_info['total_time']:.2f}s")


    def plan_entire_route(self):
        self.ai_script = []
        while len(self.waypoints) > 1:
            self._set_next_init_goal_pos()
            self._plan()
            start_t = self.ai_script[-1]["t"] + self.config.dt if self.ai_script else 0.0
            self.ai_script.extend(self._trajectory_to_ai_script(start_t=start_t))

            assert self.trajectory_states is not None
            if self._comp_waypoints(self.trajectory_states[-1], self.waypoints[0]) > 1.0:
                print(f"Warning: Last trajectory state {self.trajectory_states[-1].tolist()} "
                      f"does not match the next waypoint {list(self.waypoints[0])}.")


    def _trajectory_to_ai_script(self, start_t=0.0):
        if self.trajectory_states is None:
            raise ValueError("Trajectory states are not available. Run _plan() first.")
        ai_script = [
            {
                "x": self.trajectory_states[i, 0].item(),
                "y": self.trajectory_states[i, 1].item(),
                "z": 0.0, # TODO: use terrain height
                "t": start_t + self.config.dt * i,
            }
            for i in range(len(self.trajectory_states))
        ]
        return ai_script


    def _comp_waypoints(self, waypoint1: Float3, waypoint2: Float3) -> float:
        """Compute the Euclidean distance between two waypoints."""
        return math.sqrt(
            (waypoint1[0] - waypoint2[0]) ** 2 +
            (waypoint1[1] - waypoint2[1]) ** 2
        )


    def add_debug_trajectory_line(self, beamng):
        if self.ai_script is None:
            raise ValueError("AI script is not available. Run plan_entire_route() first.")
        return beamng.debug.add_polyline(
            [(n["x"], n["y"], n["z"]) for n in self.ai_script],
            (1, 0, 0, 1),
            cling=True,
            offset=0.1
        )
        

    @staticmethod
    def build_parser(parser: ArgumentParser) -> None:
        """Add command-line arguments accepted by ``SafeMPDController``."""

        parser.add_argument(
            "--lane-width",
            type=float,
            required=True,
            help="Lane width in metres",
        )
        parser.add_argument(
            "--spawn-pos",
            type=float,
            nargs=3,
            required=True,
            metavar=("X", "Y", "Z"),
            help="Initial vehicle position",
        )
        parser.add_argument(
            "--goal-pos",
            type=float,
            nargs=3,
            required=True,
            metavar=("X", "Y", "Z"),
            help="Goal vehicle position",
        )
        parser.add_argument(
            "--waypoint-spacing",
            type=float,
            default=math.inf,
            help="Waypoint spacing in metres. If not set, does not create intermediate waypoints.)"
        )
        parser.add_argument(
            "--time-step",
            type=float,
            default=0.25,
            help="Simulation time step in seconds (default: %(default)s)"
        )
        parser.add_argument(
            "--max-speed",
            type=float,
            default=10.0,
            help="Maximum vehicle speed in m/s (default: %(default)s)"
        )
        parser.add_argument(
            "--seed",
            type=int,
            default=42,
            help="Random seed (default: %(default)s)",
        )
        parser.add_argument(
            "--dynamics",
            default="tt2d",
            help="MBD dynamics environment (default: %(default)s)",
        )
        parser.add_argument(
            "--samples",
            type=int,
            default=1000,
            help="Number of MBD samples (default: %(default)s)",
        )
        parser.add_argument(
            "--horizon",
            type=int,
            default=500,
            help="MBD planning horizon (default: %(default)s)",
        )
        parser.add_argument(
            "--diffusion-steps",
            type=int,
            default=100,
            help="Number of diffusion steps (default: %(default)s)",
        )
        parser.add_argument(
            "--buildings",
            type=str,
            default=None,
            help="Path to a JSON file containing building polygons (default: %(default)s)",
        )
        parser.add_argument(
            "--debug",
            action="store_true",
            help="Enable MBD rendering and verbose output",
        )


__all__ = [
    "ImportedRoadGeometry",
    "RoadBoundaryObstacleBuilder",
    "RoadBoundaryObstacles",
    "SafeMPDController"
]
