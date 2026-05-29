"""
Scene plans for the simulator: named indoor/outdoor environments the synthetic
camera renders with colour and procedural texture (checkerboard floors, shaded
box geometry, sky/ceiling gradients).

A scene is a lightweight description — sky/ground palette plus a list of
axis-aligned coloured boxes (shelves, buildings, trees, furniture) — that the
``CameraRenderer`` rasterises as depth-sorted shaded quads. Not photoreal, but
coloured, textured, and visually distinct per plan so a session feels like it
happened somewhere.
"""

from __future__ import annotations

from dataclasses import dataclass, field

Color = tuple[int, int, int]


@dataclass(slots=True)
class Box:
    center: tuple[float, float, float]
    size: tuple[float, float, float]
    color: Color


@dataclass(slots=True)
class Scene:
    id: str
    name: str
    kind: str  # "indoor" | "outdoor"
    sky_top: Color
    sky_bottom: Color
    ground_color: Color
    ground_alt: Color   # second checker colour for floor texture
    grid_color: Color
    boxes: list[Box] = field(default_factory=list)
    ceiling_z: float | None = None     # indoor scenes have a ceiling plane
    ceiling_color: Color = (40, 44, 50)
    far: float = 45.0


# ---------------------------------------------------------------- builders


def _row(n: int, start: tuple[float, float], step: tuple[float, float], size, color: Color, z: float) -> list[Box]:
    boxes = []
    for i in range(n):
        cx = start[0] + step[0] * i
        cy = start[1] + step[1] * i
        boxes.append(Box((cx, cy, z + size[2] / 2), size, color))
    return boxes


def _warehouse() -> Scene:
    boxes: list[Box] = []
    shelf = (1.2, 6.0, 3.4)
    for k, x in enumerate((-8, -3, 2, 7)):
        tint = 150 + (k % 2) * 30
        boxes += _row(1, (x, 0), (0, 0), shelf, (tint, tint - 40, 70), 0.0)
    # Pallets / crates scattered.
    for (x, y) in [(-5, -7), (4, -6), (-1, 6), (6, 5)]:
        boxes.append(Box((x, y, 0.4), (1.0, 1.0, 0.8), (170, 120, 60)))
    # Perimeter walls (thin tall boxes).
    boxes.append(Box((0, -12, 2.0), (26, 0.4, 4.0), (120, 124, 130)))
    boxes.append(Box((0, 12, 2.0), (26, 0.4, 4.0), (120, 124, 130)))
    return Scene(
        id="warehouse", name="Warehouse", kind="indoor",
        sky_top=(54, 58, 66), sky_bottom=(72, 76, 84),
        ground_color=(96, 98, 104), ground_alt=(84, 86, 92), grid_color=(120, 122, 130),
        boxes=boxes, ceiling_z=4.5, ceiling_color=(46, 48, 54),
    )


def _office() -> Scene:
    boxes: list[Box] = []
    desk = (1.6, 0.9, 0.75)
    for x in (-6, -2, 2, 6):
        for y in (-3, 3):
            boxes.append(Box((x, y, 0.38), desk, (180, 180, 188)))
            boxes.append(Box((x, y, 0.95), (0.5, 0.4, 0.35), (40, 44, 52)))  # monitor
    # Glass partitions (tinted teal).
    for y in (-6, 6):
        boxes.append(Box((0, y, 1.2), (16, 0.2, 2.4), (90, 150, 160)))
    return Scene(
        id="office", name="Office", kind="indoor",
        sky_top=(60, 66, 74), sky_bottom=(82, 88, 96),
        ground_color=(70, 96, 104), ground_alt=(62, 86, 94), grid_color=(96, 120, 128),
        boxes=boxes, ceiling_z=3.2, ceiling_color=(70, 74, 80),
    )


def _city() -> Scene:
    boxes: list[Box] = []
    palette = [(110, 118, 132), (90, 96, 110), (130, 120, 110), (96, 110, 120)]
    coords = [(-9, -9), (-9, 0), (-9, 9), (0, -10), (0, 9), (9, -9), (9, 0), (9, 9), (4, -4), (-4, 4)]
    for k, (x, y) in enumerate(coords):
        h = 5.0 + (k * 2.3 % 9)
        w = 3.0 + (k % 3)
        boxes.append(Box((x, y, h / 2), (w, w, h), palette[k % len(palette)]))
    return Scene(
        id="city", name="City block (outdoor)", kind="outdoor",
        sky_top=(58, 92, 150), sky_bottom=(150, 178, 205),
        ground_color=(58, 60, 66), ground_alt=(66, 68, 74), grid_color=(150, 150, 90),
        boxes=boxes, far=60.0,
    )


def _park() -> Scene:
    boxes: list[Box] = []
    tree_coords = [(-7, -5), (-4, 4), (0, -6), (3, 6), (6, -3), (8, 4), (-8, 6), (5, 0)]
    for (x, y) in tree_coords:
        boxes.append(Box((x, y, 0.8), (0.4, 0.4, 1.6), (96, 66, 40)))      # trunk
        boxes.append(Box((x, y, 2.4), (2.4, 2.4, 2.2), (54, 132, 60)))     # canopy
    for (x, y) in [(-2, 0), (2, 2)]:
        boxes.append(Box((x, y, 0.3), (1.6, 0.5, 0.5), (140, 100, 60)))    # bench
    return Scene(
        id="park", name="Park (outdoor)", kind="outdoor",
        sky_top=(70, 120, 175), sky_bottom=(168, 196, 214),
        ground_color=(70, 120, 64), ground_alt=(60, 108, 56), grid_color=(90, 140, 80),
        boxes=boxes, far=55.0,
    )


def _open_field() -> Scene:
    boxes = [Box((x, y, 0.5), (1.0, 1.0, 1.0), (200, 90, 70)) for (x, y) in [(-6, 0), (6, 0), (0, 6), (0, -6)]]
    return Scene(
        id="open_field", name="Open field (outdoor)", kind="outdoor",
        sky_top=(64, 104, 162), sky_bottom=(158, 186, 210),
        ground_color=(78, 116, 70), ground_alt=(70, 106, 62), grid_color=(96, 132, 84),
        boxes=boxes, far=55.0,
    )


_BUILDERS = {
    "open_field": _open_field,
    "warehouse": _warehouse,
    "office": _office,
    "city": _city,
    "park": _park,
}

DEFAULT_SCENE = "open_field"


def list_scenes() -> list[dict[str, str]]:
    out = []
    for scene_id, builder in _BUILDERS.items():
        scene = builder()
        out.append({"id": scene.id, "name": scene.name, "kind": scene.kind})
    return out


def build_scene(scene_id: str | None) -> Scene:
    builder = _BUILDERS.get(scene_id or DEFAULT_SCENE, _BUILDERS[DEFAULT_SCENE])
    return builder()
