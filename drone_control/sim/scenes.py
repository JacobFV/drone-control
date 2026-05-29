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
    label: str = "object"


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


def _row(n: int, start: tuple[float, float], step: tuple[float, float], size, color: Color, z: float, label: str = "object") -> list[Box]:
    boxes = []
    for i in range(n):
        cx = start[0] + step[0] * i
        cy = start[1] + step[1] * i
        boxes.append(Box((cx, cy, z + size[2] / 2), size, color, label))
    return boxes


def _jit(base: Color, k: int, amp: int = 18) -> Color:
    """Deterministic per-index colour variation so repeated geometry isn't flat."""
    d = ((k * 37) % (2 * amp)) - amp
    return tuple(int(max(0, min(255, c + d))) for c in base)


def _warehouse() -> Scene:
    boxes: list[Box] = []
    # Storage aisles: each rack is several stacked, segmented shelf bays.
    for ai, x in enumerate((-9, -4.5, 0, 4.5, 9)):
        for by, y in enumerate(range(-8, 9, 4)):
            for level in range(3):
                z = level * 1.25
                tint = 150 + (ai % 2) * 30 - level * 12
                boxes.append(Box((x, y, z + 0.55), (1.4, 3.4, 1.1), (tint, tint - 45, 70), "shelf"))
            # boxes resting on some bays
            if (ai + by) % 2 == 0:
                boxes.append(Box((x, y, 3.9), (1.2, 1.2, 0.8), _jit((175, 130, 70), ai + by), "crate"))
    # Support columns.
    for cx in (-11, 0, 11):
        for cy in (-10, 10):
            boxes.append(Box((cx, cy, 2.25), (0.5, 0.5, 4.5), (90, 92, 98), "column"))
    # Pallet stacks + a couple of forklifts in the aisles.
    for k, (x, y) in enumerate([(-6.8, -2), (2.2, 5), (6.8, -5), (-2.2, 7)]):
        boxes.append(Box((x, y, 0.5), (1.1, 1.1, 1.0), _jit((165, 120, 60), k), "pallet"))
    for (x, y) in [(-2.2, -6), (6.8, 2)]:
        boxes.append(Box((x, y, 0.6), (1.4, 0.9, 1.2), (210, 150, 40), "forklift"))
    # Perimeter walls with loading-dock doors.
    boxes.append(Box((0, -12, 2.0), (28, 0.4, 4.0), (118, 122, 130), "wall"))
    boxes.append(Box((0, 12, 2.0), (28, 0.4, 4.0), (118, 122, 130), "wall"))
    for dx in (-8, -3, 2, 7):
        boxes.append(Box((dx, 11.8, 1.4), (2.4, 0.3, 2.8), (70, 74, 82), "dock-door"))
    return Scene(
        id="warehouse", name="Warehouse", kind="indoor",
        sky_top=(50, 54, 62), sky_bottom=(74, 78, 86),
        ground_color=(98, 100, 106), ground_alt=(86, 88, 94), grid_color=(120, 122, 130),
        boxes=boxes, ceiling_z=5.0, ceiling_color=(44, 46, 52),
    )


def _office() -> Scene:
    boxes: list[Box] = []
    # Cubicle grid: desk + monitor + chair, ringed by low partitions.
    for gi, x in enumerate((-7, -2.5, 2.5, 7)):
        for gj, y in enumerate((-5, 0, 5)):
            boxes.append(Box((x, y, 0.38), (1.7, 1.0, 0.75), (188, 188, 196), "desk"))
            boxes.append(Box((x - 0.4, y, 1.0), (0.55, 0.45, 0.4), (38, 42, 50), "monitor"))
            boxes.append(Box((x + 0.5, y, 0.45), (0.6, 0.6, 0.9), _jit((60, 70, 90), gi + gj), "chair"))
            boxes.append(Box((x, y - 0.9, 0.7), (1.9, 0.12, 1.3), (120, 170, 175), "partition"))
    # Meeting room (table) + plants in corners.
    boxes.append(Box((0, 8.5, 0.4), (3.6, 1.4, 0.8), (150, 120, 90), "table"))
    for (x, y) in [(-9, 8.5), (9, 8.5), (-9, -8), (9, -8)]:
        boxes.append(Box((x, y, 0.7), (0.7, 0.7, 1.4), (60, 130, 70), "plant"))
    # Glass curtain walls.
    for y in (-9, 9):
        boxes.append(Box((0, y, 1.3), (20, 0.18, 2.6), (96, 156, 166), "glass-wall"))
    return Scene(
        id="office", name="Office", kind="indoor",
        sky_top=(58, 64, 72), sky_bottom=(86, 92, 100),
        ground_color=(70, 96, 104), ground_alt=(62, 86, 94), grid_color=(96, 120, 128),
        boxes=boxes, ceiling_z=3.2, ceiling_color=(74, 78, 84),
    )


def _city() -> Scene:
    boxes: list[Box] = []
    palette = [(112, 120, 134), (92, 98, 112), (132, 122, 112), (98, 112, 122), (120, 110, 128)]
    # Buildings on a block grid, each a setback stack of 2-3 masses.
    k = 0
    for x in (-11, -4, 4, 11):
        for y in (-11, -4, 4, 11):
            if abs(x) < 5 and abs(y) < 5:
                continue  # central plaza
            base_h = 6.0 + ((x * 3 + y * 7) % 14)
            w = 3.2 + (k % 3) * 0.6
            color = palette[k % len(palette)]
            boxes.append(Box((x, y, base_h / 2), (w, w, base_h), color, "building"))
            boxes.append(Box((x, y, base_h + 1.6), (w * 0.66, w * 0.66, 3.2), _jit(color, k, 12), "building"))
            if k % 2 == 0:
                boxes.append(Box((x, y, base_h + 4.0), (0.4, 0.4, 1.8), (200, 80, 70), "antenna"))
            k += 1
    # Streets: cars along the avenues + street lamps.
    for i, x in enumerate(range(-9, 10, 3)):
        boxes.append(Box((x, 0, 0.35), (1.0, 2.0, 0.7), _jit((150, 60, 60), i, 40), "car"))
        boxes.append(Box((0, x, 0.35), (2.0, 1.0, 0.7), _jit((60, 80, 150), i, 40), "car"))
    for (x, y) in [(-2.5, -2.5), (2.5, 2.5), (-2.5, 2.5), (2.5, -2.5)]:
        boxes.append(Box((x, y, 1.4), (0.2, 0.2, 2.8), (60, 62, 68), "lamp"))
    return Scene(
        id="city", name="City block (outdoor)", kind="outdoor",
        sky_top=(56, 90, 150), sky_bottom=(152, 180, 206),
        ground_color=(56, 58, 64), ground_alt=(66, 68, 74), grid_color=(150, 150, 90),
        boxes=boxes, far=70.0,
    )


def _park() -> Scene:
    boxes: list[Box] = []
    # Trees of varied size (trunk + 1-2 canopy tiers).
    tree_coords = [(-9, -6), (-5, 4), (0, -7), (3, 7), (7, -4), (9, 5), (-9, 7), (5, 0), (-3, -3), (8, -8)]
    for k, (x, y) in enumerate(tree_coords):
        scale = 0.7 + (k % 4) * 0.35
        boxes.append(Box((x, y, 0.8 * scale), (0.4, 0.4, 1.6 * scale), (96, 66, 40), "tree"))
        boxes.append(Box((x, y, 2.2 * scale), (2.4 * scale, 2.4 * scale, 2.0 * scale), _jit((54, 132, 60), k, 24), "tree"))
        if k % 3 == 0:
            boxes.append(Box((x, y, 3.4 * scale), (1.5 * scale, 1.5 * scale, 1.3 * scale), _jit((64, 150, 70), k, 20), "tree"))
    # Pond (flat blue slab), paths, benches, a small playground.
    boxes.append(Box((-2, 2, 0.05), (5.0, 4.0, 0.1), (70, 120, 165), "pond"))
    for x in range(-9, 10, 3):
        boxes.append(Box((x, -1, 0.04), (2.6, 1.2, 0.08), (170, 150, 110), "path"))
    for (x, y) in [(-4, -4), (4, 4), (6, -2)]:
        boxes.append(Box((x, y, 0.3), (1.6, 0.5, 0.5), (140, 100, 60), "bench"))
    boxes.append(Box((7, 7, 0.6), (1.6, 1.6, 1.2), (210, 120, 60), "playground"))
    return Scene(
        id="park", name="Park (outdoor)", kind="outdoor",
        sky_top=(70, 120, 175), sky_bottom=(170, 198, 216),
        ground_color=(70, 122, 64), ground_alt=(60, 110, 56), grid_color=(90, 140, 80),
        boxes=boxes, far=60.0,
    )


def _construction() -> Scene:
    boxes: list[Box] = []
    # A building under construction: floor slabs on a column grid, rising.
    for floor in range(4):
        z = floor * 2.2
        boxes.append(Box((-3, 0, z + 0.1), (7.0, 7.0, 0.2), _jit((150, 150, 156), floor, 10), "slab"))
        for cx in (-6, 0):
            for cy in (-3, 3):
                boxes.append(Box((cx, cy, z + 1.1), (0.4, 0.4, 2.2), (120, 110, 90), "column"))
    # Tower crane (mast + jib + counter-jib).
    boxes.append(Box((8, -8, 7.0), (0.6, 0.6, 14.0), (220, 180, 40), "crane"))
    boxes.append(Box((4, -8, 13.6), (9.0, 0.5, 0.5), (220, 180, 40), "crane"))
    boxes.append(Box((10.5, -8, 13.6), (2.5, 0.5, 0.5), (200, 160, 40), "crane"))
    # Scaffolding, material stacks, site cabins, excavator.
    for y in range(-6, 7, 2):
        boxes.append(Box((1, y, 1.5), (0.15, 0.15, 3.0), (180, 150, 60), "scaffold"))
    for k, (x, y) in enumerate([(8, 3), (6, 6), (9, 6)]):
        boxes.append(Box((x, y, 0.5), (1.6, 1.0, 1.0), _jit((150, 120, 80), k), "materials"))
    boxes.append(Box((-9, -7, 1.0), (2.6, 2.0, 2.0), (210, 130, 50), "site-cabin"))
    boxes.append(Box((-8, 6, 0.7), (2.2, 1.2, 1.4), (220, 190, 40), "excavator"))
    return Scene(
        id="construction", name="Construction site (outdoor)", kind="outdoor",
        sky_top=(120, 130, 150), sky_bottom=(186, 192, 200),
        ground_color=(126, 110, 88), ground_alt=(116, 100, 80), grid_color=(150, 130, 90),
        boxes=boxes, far=70.0,
    )


def _atrium() -> Scene:
    boxes: list[Box] = []
    # Multi-level indoor atrium: balconies ringing an open central void.
    for level in range(1, 4):
        z = level * 2.4
        for x in (-9, 9):
            boxes.append(Box((x, 0, z), (0.6, 18, 0.3), _jit((150, 152, 160), level, 10), "balcony"))
        for y in (-9, 9):
            boxes.append(Box((0, y, z), (18, 0.6, 0.3), _jit((150, 152, 160), level, 10), "balcony"))
        for x in (-9, 9):
            boxes.append(Box((x, 0, z + 0.9), (0.1, 18, 0.9), (120, 170, 180), "railing"))
    # Escalator block, info desk, planters, kiosks on the ground floor.
    boxes.append(Box((0, 0, 1.2), (3.0, 6.0, 2.4), (110, 116, 128), "escalator"))
    boxes.append(Box((0, -7, 0.5), (3.0, 1.2, 1.0), (170, 150, 120), "desk"))
    for (x, y) in [(-6, -6), (6, 6), (-6, 6), (6, -6)]:
        boxes.append(Box((x, y, 0.6), (1.2, 1.2, 1.2), (60, 130, 72), "planter"))
    for (x, y) in [(-7, 0), (7, 0)]:
        boxes.append(Box((x, y, 0.9), (1.6, 2.2, 1.8), _jit((120, 90, 160), int(x)), "kiosk"))
    return Scene(
        id="atrium", name="Atrium (indoor)", kind="indoor",
        sky_top=(70, 76, 88), sky_bottom=(120, 126, 138),
        ground_color=(120, 122, 130), ground_alt=(106, 108, 118), grid_color=(140, 142, 150),
        boxes=boxes, ceiling_z=8.0, ceiling_color=(60, 64, 72),
    )


def _open_field() -> Scene:
    boxes: list[Box] = []
    # Markers + a perimeter fence + a few hay bales / a windsock pole.
    for (x, y) in [(-6, 0), (6, 0), (0, 6), (0, -6)]:
        boxes.append(Box((x, y, 0.5), (1.0, 1.0, 1.0), (200, 90, 70), "marker"))
    for i in range(-12, 13, 3):
        boxes.append(Box((i, -12, 0.6), (0.15, 0.15, 1.2), (140, 120, 90), "fence-post"))
        boxes.append(Box((i, 12, 0.6), (0.15, 0.15, 1.2), (140, 120, 90), "fence-post"))
    for k, (x, y) in enumerate([(-4, 3), (3, -2), (5, 4)]):
        boxes.append(Box((x, y, 0.5), (1.4, 0.9, 1.0), _jit((180, 160, 80), k), "hay-bale"))
    boxes.append(Box((0, 0, 1.5), (0.2, 0.2, 3.0), (200, 200, 210), "windsock"))
    return Scene(
        id="open_field", name="Open field (outdoor)", kind="outdoor",
        sky_top=(64, 104, 162), sky_bottom=(160, 188, 212),
        ground_color=(78, 118, 70), ground_alt=(70, 108, 62), grid_color=(96, 132, 84),
        boxes=boxes, far=60.0,
    )


_BUILDERS = {
    "open_field": _open_field,
    "warehouse": _warehouse,
    "office": _office,
    "city": _city,
    "park": _park,
    "construction": _construction,
    "atrium": _atrium,
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
