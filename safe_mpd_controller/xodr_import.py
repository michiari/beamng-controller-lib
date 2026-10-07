import math
from collections import defaultdict
import xml.etree.ElementTree as ET

from beamngpy import MeshRoad, Scenario, Road
from beamngpy.tools import OpenDriveImporter
from beamngpy.tools.opendrive_import import Road as OpenDriveRoad


class OpenDriveExtendedImporter(OpenDriveImporter):
    @staticmethod
    def extract_road_topology(filename):
        """Return OpenDRIVE road links keyed by their source road IDs."""

        root = ET.parse(filename).getroot()
        topology = {}
        for road in root.findall("road"):
            road_id = road.get("id")
            if road_id is None:
                continue

            road_topology = {"junction": road.get("junction", "-1")}
            for relation in ("predecessor", "successor"):
                link = road.find(f"./link/{relation}")
                if link is None:
                    continue
                road_topology[relation] = {
                    "element_type": link.get("elementType"),
                    "element_id": link.get("elementId"),
                    "contact_point": link.get("contactPoint"),
                }
            topology[str(road_id)] = road_topology
        return topology

    @staticmethod
    def node_distance(a, b):
        return math.sqrt(
            (b[0] - a[0]) ** 2
            + (b[1] - a[1]) ** 2
            + (b[2] - a[2]) ** 2
        )

    @staticmethod
    def node_distance_xy(a, b):
        return math.hypot(b[0] - a[0], b[1] - a[1])

    @staticmethod
    def node_chain_distances(nodes):
        distances = [0.0]
        for start, end in zip(nodes[:-1], nodes[1:]):
            distances.append(
                distances[-1] + OpenDriveExtendedImporter.node_distance_xy(start, end)
            )
        return distances

    @staticmethod
    def smooth_values_by_distance(values, distances, window_m, passes):
        if len(values) < 3 or window_m <= 0.0 or passes <= 0:
            return values

        radius_m = max(0.1, float(window_m) * 0.5)
        smoothed = [float(value) for value in values]

        for _ in range(int(passes)):
            next_values = smoothed.copy()
            left_index = 0
            right_index = 0
            for index, distance in enumerate(distances):
                while (
                    left_index < len(distances)
                    and distances[left_index] < distance - radius_m
                ):
                    left_index += 1
                while (
                    right_index < len(distances)
                    and distances[right_index] <= distance + radius_m
                ):
                    right_index += 1

                total_weight = 0.0
                total_value = 0.0

                for other_index in range(left_index, right_index):
                    other_distance = distances[other_index]
                    delta = abs(other_distance - distance)
                    t = delta / radius_m
                    weight = 0.5 + 0.5 * math.cos(math.pi * t)
                    total_weight += weight
                    total_value += smoothed[other_index] * weight

                if total_weight > 0.0:
                    next_values[index] = total_value / total_weight

            smoothed = next_values

        return smoothed

    @staticmethod
    def smooth_node_elevations(nodes, window_m=0.0, passes=0):
        if len(nodes) < 3 or window_m <= 0.0 or passes <= 0:
            return nodes

        distances = OpenDriveExtendedImporter.node_chain_distances(nodes)
        if distances[-1] <= 1e-9:
            return nodes

        z_values = [float(node[2]) for node in nodes]
        smoothed_z = OpenDriveExtendedImporter.smooth_values_by_distance(
            z_values,
            distances,
            window_m,
            passes,
        )

        smoothed_nodes = []
        for node, z in zip(nodes, smoothed_z):
            values = list(node)
            values[2] = z
            smoothed_nodes.append(tuple(values))
        return smoothed_nodes

    @staticmethod
    def catmull_rom_value(p0, p1, p2, p3, t, index):
        return 0.5 * (
            2.0 * p1[index]
            + (-p0[index] + p2[index]) * t
            + (2.0 * p0[index] - 5.0 * p1[index] + 4.0 * p2[index] - p3[index])
            * t ** 2
            + (-p0[index] + 3.0 * p1[index] - 3.0 * p2[index] + p3[index])
            * t ** 3
        )

    @staticmethod
    def smooth_nodes(nodes, spacing_m=2.0, strength=0.65):
        if len(nodes) < 3 or strength <= 0.0:
            return nodes

        spacing_m = max(0.25, float(spacing_m))
        strength = max(0.0, min(1.0, float(strength)))
        smoothed = []

        for index in range(len(nodes) - 1):
            p0 = nodes[max(0, index - 1)]
            p1 = nodes[index]
            p2 = nodes[index + 1]
            p3 = nodes[min(len(nodes) - 1, index + 2)]

            segment_length = OpenDriveExtendedImporter.node_distance(p1, p2)
            steps = max(1, int(math.ceil(segment_length / spacing_m)))

            for step in range(steps):
                if index > 0 and step == 0:
                    continue

                t = step / steps
                values = []

                for value_index in range(3):
                    linear = p1[value_index] + (p2[value_index] - p1[value_index]) * t
                    curved = OpenDriveExtendedImporter.catmull_rom_value(
                        p0,
                        p1,
                        p2,
                        p3,
                        t,
                        value_index,
                    )
                    values.append(linear + (curved - linear) * strength)

                width = p1[3] + (p2[3] - p1[3]) * t
                smoothed.append((values[0], values[1], values[2], width))

        smoothed.append(nodes[-1])
        return smoothed

    @staticmethod
    def apply_height_sampler_to_nodes(nodes, height_sampler, z_offset_m=0.0):
        if height_sampler is None:
            return nodes

        elevated = []
        nodes = [node[:4] for node in nodes]
        for x, y, _z, width in nodes:
            elevated.append((x, y, float(height_sampler(x, y)) + z_offset_m, width))
        return elevated

    @staticmethod
    def apply_height_sampler_to_roads(roads, height_sampler, z_offset_m=0.0):
        if height_sampler is None:
            return roads

        elevated_roads = []
        for road in roads:
            road.nodes = OpenDriveExtendedImporter.apply_height_sampler_to_nodes(
                road.nodes,
                height_sampler,
                z_offset_m=z_offset_m,
            )
            elevated_roads.append(road)
        return elevated_roads

    @staticmethod
    def smooth_road_elevations(roads, window_m=0.0, passes=0):
        if window_m <= 0.0 or passes <= 0:
            return roads

        smoothed_roads = []
        for road in roads:
            road.nodes = OpenDriveExtendedImporter.smooth_node_elevations(
                road.nodes,
                window_m=window_m,
                passes=passes,
            )
            smoothed_roads.append(road)
        return smoothed_roads

    @staticmethod
    def import_xodr(filename, scenario: Scenario, road_properties):

        road_topology = OpenDriveExtendedImporter.extract_road_topology(filename)

        # Extract the road data primitives from the OpenDrive file.
        print("Extracting road data from file...")
        lines, arcs, spirals, polys, cubics = OpenDriveImporter.extract_road_data(
            filename
        )
        print(
            "Primitives to import:  lines:",
            len(lines),
            "; arcs:",
            len(arcs),
            "; spirals:",
            len(spirals),
            "; explicit cubics:",
            len(polys),
            "; parametric cubics:",
            len(cubics),
        )

        # Generate separate R^3 road polylines from each imported OpenDrive primitive data.
        print("Generating geometric primitives...")

        roads = []
        road_groups = defaultdict(list)

        for prim in lines + arcs + spirals + polys + cubics:
            road_groups[prim.id].append(prim)

        for road_id, prims in road_groups.items():
            prims = sorted(prims, key=lambda p: p.s)

            nodes = []

            for prim in prims:
                part = prim.discretize()

                if not part:
                    continue

                if nodes:
                    # remove duplicated join node if present
                    x0, y0, z0, *_ = part[0]
                    x1, y1, z1, *_ = nodes[-1]

                    if ((x0 - x1) ** 2 + (y0 - y1) ** 2 + (z0 - z1) ** 2) ** 0.5 < 0.05:
                        part = part[1:]

                nodes.extend(part)

            roads.append(OpenDriveRoad(f"imported_{road_id}", nodes))

        # Perform offset re-computation to get the correct road reference line for BeamNG.
        print("Adding road lateral offsetting...")
        roads = OpenDriveImporter.add_lateral_offset(roads)

        # Adjust the elevation of the road polylines so they can be rendered appropriately in the BeamNG world.
        surface_type = road_properties.get("surface_type", "mesh")
        height_sampler = road_properties.get("height_sampler")
        road_height_offset_m = road_properties.get("height_sampler_z_offset_m", 0.0)
        if height_sampler is not None:
            print("Sampling road elevation from terrain...")
            roads = OpenDriveExtendedImporter.apply_height_sampler_to_roads(
                roads,
                height_sampler,
                z_offset_m=road_height_offset_m,
            )
        else:
            print("Adjusting global elevation...")
            default_min_elevation = 5.0 if surface_type == "mesh" else 0.0
            roads = OpenDriveImporter.adjust_elevation(
                roads,
                min_elev=road_properties.get("min_elevation", default_min_elevation),
            )

        elevation_smoothing_window_m = float(
            road_properties.get("elevation_smoothing_window_m", 0.0)
        )
        elevation_smoothing_passes = int(
            road_properties.get("elevation_smoothing_passes", 0)
        )
        if elevation_smoothing_window_m > 0.0 and elevation_smoothing_passes > 0:
            print(
                "Smoothing road elevation profile "
                f"({elevation_smoothing_window_m:.1f} m window, "
                f"{elevation_smoothing_passes} passes)..."
            )
            roads = OpenDriveExtendedImporter.smooth_road_elevations(
                roads,
                window_m=elevation_smoothing_window_m,
                passes=elevation_smoothing_passes,
            )

        # Create the all the roads from the road polyline data (which came from various OpenDrive primitive evaluators).
        print("Loading import in scenario...")
        imported_roads = []

        for i, r in enumerate(roads):
            base_nodes = [tuple(x[:4]) for x in r.nodes]
            road_id = f"road_{i}"
            source_road_id = r.name.removeprefix("imported_")
            render_nodes = base_nodes

            if road_properties.get("smooth_render_nodes", False):
                render_nodes = OpenDriveExtendedImporter.smooth_nodes(
                    base_nodes,
                    spacing_m=road_properties.get("smoothing_spacing_m", 2.0),
                    strength=road_properties.get("smoothing_strength", 0.65),
                )
                render_nodes = OpenDriveExtendedImporter.apply_height_sampler_to_nodes(
                    render_nodes,
                    height_sampler,
                    z_offset_m=road_height_offset_m,
                )
                render_nodes = OpenDriveExtendedImporter.smooth_node_elevations(
                    render_nodes,
                    window_m=elevation_smoothing_window_m,
                    passes=elevation_smoothing_passes,
                )

            if surface_type == "mesh":
                mesh_depth = max(
                    0.0,
                    float(road_properties.get("mesh_depth", 0.01)),
                )
                asphalt_nodes = [
                    (x, y, z + 0.02, width, mesh_depth)
                    for x, y, z, width in render_nodes
                ]
                asphalt = MeshRoad(
                    top_material=road_properties.get("material", "road_asphalt_light"),
                    bottom_material=road_properties.get("bottom_material"),
                    side_material=road_properties.get("side_material"),
                    rid=road_id,
                    default_width=road_properties["default_width"],
                    default_depth=mesh_depth,
                    texture_length=road_properties["texture_length"],
                    break_angle=road_properties.get("break_angle", 45),
                    width_subdivisions=road_properties.get("width_subdivisions", 1),
                )
                asphalt.render_priority = int(road_properties.get("render_priority", 8))
                asphalt.add_nodes(*asphalt_nodes)
                scenario.add_mesh_road(asphalt)
            else:
                asphalt_nodes = [
                    (x, y, z + 0.02, width)
                    for x, y, z, width in render_nodes
                ]
                asphalt = Road(
                    material=road_properties.get("material", "road_asphalt_light"),
                    rid=road_id,
                    default_width=road_properties["default_width"],
                    drivability=road_properties["drivability"],
                    texture_length=road_properties["texture_length"],
                    render_priority=road_properties.get("render_priority", 8),
                    interpolate=road_properties.get("interpolate", True),
                    break_angle=road_properties.get("break_angle", 45),
                    smoothness=road_properties.get("smoothness", 0.5),
                    over_objects=road_properties.get("over_objects", True),
                )
                asphalt.add_nodes(*asphalt_nodes)
                scenario.add_road(asphalt)
            imported_roads.append({
                "rid": road_id,
                "source_road_id": source_road_id,
                "nodes": render_nodes,
                "source_nodes": base_nodes,
                **road_topology.get(source_road_id, {}),
            })

        print("Import complete.")
        return imported_roads
