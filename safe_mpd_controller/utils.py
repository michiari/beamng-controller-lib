import json
from pathlib import Path
from shapely.geometry import shape

def load_building_polygons(filename):
    """Load building polygons and offsets written by
    :func:`dump_building_polygons`.

    :param filename: Path of the JSON file to read.
    :return: ``(polygons, offsets)`` in the same form as returned by
        :func:`extract_building_polygons`.
    """
    payload = json.loads(Path(filename).read_text(encoding="utf-8"))
    polygons = [shape(geometry) for geometry in payload["polygons"]]
    offsets = [tuple(offset) for offset in payload["offsets"]]

    if len(polygons) != len(offsets):
        raise ValueError("The number of building polygons and offsets must match")

    return polygons, offsets
