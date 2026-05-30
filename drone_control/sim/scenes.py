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

import math
from dataclasses import dataclass, field

from .flow import FlowSpec
from .particles import ParticleSpec
from .smoke import FireSpec, SmokeSpec

Color = tuple[int, int, int]


@dataclass(slots=True)
class RigidSpec:
    """Scene-authored description of one free rigid body (pure data)."""

    label: str
    color: Color
    size: tuple[float, float, float]                    # box full dimensions (m)
    mass: float = 0.5                                   # kg
    pos: tuple[float, float, float] = (0.0, 0.0, 1.0)   # initial center
    vel: tuple[float, float, float] = (0.0, 0.0, 0.0)
    restitution: float = 0.35                           # bounce on ground contact
    friction: float = 0.5                               # tangential damping on contact
    drag_cd: float = 1.05
    spin: tuple[float, float, float] = (0.0, 0.0, 0.0)  # initial body omega (rad/s)


@dataclass(slots=True)
class ClothSpec:
    """A deformable cloth panel (PyBullet soft body) — a hanging flag, banner,
    curtain, or garment pinned along its top edge. Pure data (no PyBullet
    import) so scene-building stays dependency-light."""

    anchor: tuple[float, float, float]      # world point the top edge pins to
    color: Color = (200, 80, 80)
    width: float = 1.4
    height: float = 1.0
    mass: float = 0.1
    label: str = "cloth"
    kind: str = "flag"                       # flag | banner | curtain | garment
    plane: str = "xz"                        # plane the panel hangs in (xz | yz)
    stiffness: float = 40.0


@dataclass(slots=True)
class ClothInstanceGroup:
    """Cloth instancing: ONE simulated master panel rendered as many instances.

    Only ``master`` is simulated (PyBullet); every placement re-uses its live
    deformed mesh with a per-instance sway, so a shop can show hundreds of
    swaying garments from a handful of soft-body sims."""

    master: ClothSpec
    anchors: list[tuple[float, float, float]] = field(default_factory=list)
    colors: list[Color] = field(default_factory=list)
    label: str = "garment"

    def placements(self) -> list[tuple]:
        """(anchor, color, label, phase) per instance — deterministic phases."""
        out = []
        for i, anchor in enumerate(self.anchors):
            color = self.colors[i % len(self.colors)] if self.colors else self.master.color
            phase = (i * 0.61803398875) * 2.0 * math.pi  # golden-ratio spread
            out.append((anchor, color, self.label, phase))
        return out


@dataclass(slots=True)
class Box:
    center: tuple[float, float, float]
    size: tuple[float, float, float]
    color: Color
    label: str = "object"


@dataclass(slots=True)
class DynamicSpec:
    """A moving scene object. ``motion`` + ``params`` define a deterministic
    parametric path in time so positions are reproducible across renders."""

    label: str
    color: Color
    size: tuple[float, float, float]
    motion: str                      # "line" | "circle" | "patrol"
    params: dict
    z: float = 0.5

    def position_at(self, t: float) -> tuple[float, float, float]:
        p = self.params
        if self.motion == "circle":
            w = 2 * math.pi / max(1e-3, p["period"])
            ang = w * t + p.get("phase", 0.0)
            return (p["cx"] + p["r"] * math.cos(ang), p["cy"] + p["r"] * math.sin(ang), self.z)
        if self.motion == "line":
            a, b, period = p["a"], p["b"], p["period"]
            frac = (t % period) / period
            tri = 2 * frac if frac < 0.5 else 2 * (1 - frac)   # ping-pong 0..1..0
            return (a[0] + (b[0] - a[0]) * tri, a[1] + (b[1] - a[1]) * tri, self.z)
        if self.motion == "patrol":
            pts = p["points"]
            speed = p.get("speed", 1.0)
            segs = [(pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts))]
            seglens = [math.dist(s, e) for s, e in segs]
            total = sum(seglens) or 1.0
            d = (t * speed) % total
            for (s, e), L in zip(segs, seglens):
                if d <= L:
                    f = d / (L or 1.0)
                    return (s[0] + (e[0] - s[0]) * f, s[1] + (e[1] - s[1]) * f, self.z)
                d -= L
            return (pts[0][0], pts[0][1], self.z)
        return (p.get("cx", 0.0), p.get("cy", 0.0), self.z)


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
    dynamics: list[DynamicSpec] = field(default_factory=list)   # moving objects
    flows: list[FlowSpec] = field(default_factory=list)          # airflow primitives
    rigids: list[RigidSpec] = field(default_factory=list)        # free rigid bodies
    cloths: list[ClothSpec] = field(default_factory=list)        # deformable cloth
    cloth_groups: list[ClothInstanceGroup] = field(default_factory=list)  # instanced cloth
    particles: list[ParticleSpec] = field(default_factory=list)  # dust tracers
    smokes: list[SmokeSpec] = field(default_factory=list)        # volumetric smoke
    fires: list[FireSpec] = field(default_factory=list)          # fire + smoke columns
    ceiling_z: float | None = None     # indoor scenes have a ceiling plane
    ceiling_color: Color = (40, 44, 50)
    far: float = 45.0
    fog_density: float = 0.0           # atmospheric depth fade (0 = none)


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
    dynamics = [
        # Forklifts shuttling down two aisles + an AGV crossing.
        DynamicSpec("forklift", (220, 160, 40), (1.2, 1.0, 1.4), "line",
                    {"a": (-2.2, -10), "b": (-2.2, 10), "period": 14.0}, z=0.7),
        DynamicSpec("forklift", (210, 120, 40), (1.2, 1.0, 1.4), "line",
                    {"a": (6.8, 10), "b": (6.8, -10), "period": 16.0}, z=0.7),
        DynamicSpec("agv", (90, 170, 200), (0.9, 0.9, 0.5), "line",
                    {"a": (-11, 0), "b": (11, 0), "period": 12.0}, z=0.3),
    ]
    # Big roll-up door draft blowing down the central aisle + a ceiling HVAC fan.
    flows = [
        FlowSpec("wind", {"dir": (0.0, -1.4, 0.0), "gust": 0.4, "period": 10.0, "turbulence": 0.3, "shear_ref": 4.0}),
        FlowSpec("fan", {"pos": (0.0, 11.6, 3.2), "dir": (0.0, -1.0, -0.1), "speed": 6.5, "radius": 1.4, "reach": 14.0, "spread": 0.7}, label="dock-fan"),
    ]
    # A loose crate sitting in the dock-fan's draft.
    rigids = [RigidSpec("crate", (175, 130, 70), (0.9, 0.9, 0.9), mass=0.7, pos=(0.0, 8.0, 0.45))]
    return Scene(
        id="warehouse", name="Warehouse", kind="indoor",
        sky_top=(50, 54, 62), sky_bottom=(74, 78, 86),
        ground_color=(98, 100, 106), ground_alt=(86, 88, 94), grid_color=(120, 122, 130),
        boxes=boxes, dynamics=dynamics, flows=flows, rigids=rigids, ceiling_z=5.0, ceiling_color=(44, 46, 52),
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
    flows = [FlowSpec("fan", {"pos": (-9.0, 0.0, 2.6), "dir": (1.0, 0.0, -0.1), "speed": 4.5, "radius": 1.1, "reach": 12.0, "spread": 0.6}, label="ac-vent")]
    return Scene(
        id="office", name="Office", kind="indoor",
        sky_top=(58, 64, 72), sky_bottom=(86, 92, 100),
        ground_color=(70, 96, 104), ground_alt=(62, 86, 94), grid_color=(96, 120, 128),
        boxes=boxes, flows=flows, ceiling_z=3.2, ceiling_color=(74, 78, 84),
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
    # Traffic: cars driving the avenues (some along x, some along y), at offsets.
    dynamics = [
        DynamicSpec("car", (200, 70, 60), (1.0, 2.0, 0.7), "line", {"a": (-1.5, -12), "b": (-1.5, 12), "period": 12.0}, z=0.35),
        DynamicSpec("car", (70, 110, 210), (1.0, 2.0, 0.7), "line", {"a": (1.5, 12), "b": (1.5, -12), "period": 14.0}, z=0.35),
        DynamicSpec("car", (230, 200, 70), (2.0, 1.0, 0.7), "line", {"a": (-12, 1.5), "b": (12, 1.5), "period": 13.0}, z=0.35),
        DynamicSpec("car", (90, 200, 120), (2.0, 1.0, 0.7), "line", {"a": (12, -1.5), "b": (-12, -1.5), "period": 15.0}, z=0.35),
        DynamicSpec("bus", (220, 160, 50), (2.6, 1.2, 1.1), "line", {"a": (-1.5, 12), "b": (-1.5, -12), "period": 22.0}, z=0.55),
    ]
    # Wind channels down the avenues (urban canyon) with strong gusts; a banner
    # flies off one of the central buildings.
    flows = [
        FlowSpec("wind", {"dir": (0.6, 4.2, 0.0), "gust": 0.5, "period": 8.5, "turbulence": 1.1, "shear_ref": 6.0}),
    ]
    cloths = [ClothSpec(anchor=(4.0, 4.0, 9.0), width=3.2, height=1.8, color=(80, 120, 210), label="banner", kind="banner", plane="yz")]
    rigids = [
        RigidSpec("debris", (180, 180, 175), (0.4, 0.4, 0.25), mass=0.08, pos=(0.0, -2.0, 0.2)),
        RigidSpec("debris", (160, 150, 140), (0.5, 0.3, 0.3), mass=0.1, pos=(-1.0, 1.0, 0.2), spin=(0.0, 0.0, 0.6)),
    ]
    particles = [ParticleSpec("dust", (0.0, -4.0, 0.3), count=50, color=(150, 150, 156), spawn_radius=6.0, lifetime=4.0)]
    return Scene(
        id="city", name="City block (outdoor)", kind="outdoor",
        sky_top=(56, 90, 150), sky_bottom=(152, 180, 206),
        ground_color=(56, 58, 64), ground_alt=(66, 68, 74), grid_color=(150, 150, 90),
        boxes=boxes, dynamics=dynamics, flows=flows, cloths=cloths, rigids=rigids, particles=particles, far=70.0, fog_density=0.016,
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
    dynamics = [
        DynamicSpec("person", (210, 170, 150), (0.5, 0.5, 1.7), "patrol",
                    {"points": [(-8, -1), (8, -1), (8, 5), (-8, 5)], "speed": 1.6}, z=0.85),
        DynamicSpec("cyclist", (90, 90, 200), (0.6, 1.4, 1.2), "circle",
                    {"cx": 0.0, "cy": 0.0, "r": 8.0, "period": 20.0}, z=0.6),
        DynamicSpec("dog", (180, 150, 90), (0.7, 0.4, 0.5), "patrol",
                    {"points": [(-6, 2), (-2, -2), (2, 3)], "speed": 2.2}, z=0.25),
    ]
    flows = [
        FlowSpec("wind", {"dir": (1.8, 1.2, 0.0), "gust": 0.6, "period": 9.0, "turbulence": 0.6, "shear_ref": 4.0}),
    ]
    return Scene(
        id="park", name="Park (outdoor)", kind="outdoor",
        sky_top=(70, 120, 175), sky_bottom=(170, 198, 216),
        ground_color=(70, 122, 64), ground_alt=(60, 110, 56), grid_color=(90, 140, 80),
        boxes=boxes, dynamics=dynamics, flows=flows, far=60.0, fog_density=0.014,
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
    dynamics = [
        # The crane hook sweeps; a dump truck loops the haul road.
        DynamicSpec("hook", (240, 200, 60), (0.4, 0.4, 0.6), "circle",
                    {"cx": 6.0, "cy": -8.0, "r": 4.0, "period": 16.0}, z=6.0),
        DynamicSpec("truck", (180, 150, 60), (1.6, 1.0, 1.1), "line",
                    {"a": (-11, -10), "b": (11, -10), "period": 18.0}, z=0.6),
    ]
    # Open-site wind, a big industrial fan blowing across the deck, and a dusty
    # thermal updraft rising through the open structure.
    flows = [
        FlowSpec("wind", {"dir": (2.6, -1.4, 0.0), "gust": 0.6, "period": 7.5, "turbulence": 1.0, "shear_ref": 6.0}),
        FlowSpec("fan", {"pos": (-11.0, 0.0, 1.4), "dir": (1.0, 0.0, 0.15), "speed": 9.0, "radius": 1.6, "reach": 16.0, "spread": 0.8}, label="site-fan"),
        FlowSpec("updraft", {"cx": -3.0, "cy": 0.0, "radius": 4.5, "speed": 2.6, "top": 12.0}),
    ]
    cloths = [ClothSpec(anchor=(8.2, -8.0, 13.8), width=2.0, height=1.1, color=(230, 200, 60), label="crane-flag", kind="flag", plane="xz")]
    # Loose materials the fan/wind can shove, a burn-barrel fire + smoke, site dust.
    rigids = [
        RigidSpec("pallet", (170, 130, 75), (1.0, 0.9, 0.4), mass=0.6, pos=(-7.0, 1.5, 0.3), spin=(0.0, 0.0, 0.3)),
        RigidSpec("debris", (150, 145, 135), (0.5, 0.5, 0.4), mass=0.2, pos=(-9.0, 0.5, 0.3)),
        RigidSpec("debris", (165, 150, 120), (0.4, 0.6, 0.3), mass=0.18, pos=(-8.0, -1.0, 0.3), spin=(0.4, 0.2, 0.0)),
    ]
    particles = [ParticleSpec("dust", (-3.0, 0.0, 0.2), count=60, color=(170, 150, 120), spawn_radius=7.0, lifetime=4.0)]
    fires = [FireSpec((-8.0, 6.0, 0.4), radius=0.5, height=1.4, intensity=0.9, smoke_rate=45, flame_rate=22, rise=3.2, label="burn-barrel")]
    return Scene(
        id="construction", name="Construction site (outdoor)", kind="outdoor",
        sky_top=(120, 130, 150), sky_bottom=(186, 192, 200),
        ground_color=(126, 110, 88), ground_alt=(116, 100, 80), grid_color=(150, 130, 90),
        boxes=boxes, dynamics=dynamics, flows=flows, cloths=cloths, rigids=rigids, particles=particles, fires=fires, far=70.0, fog_density=0.02,
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
    # Warm air rising through the open central void (stack effect).
    flows = [FlowSpec("updraft", {"cx": 0.0, "cy": 0.0, "radius": 5.0, "speed": 2.2, "top": 8.0})]
    return Scene(
        id="atrium", name="Atrium (indoor)", kind="indoor",
        sky_top=(70, 76, 88), sky_bottom=(120, 126, 138),
        ground_color=(120, 122, 130), ground_alt=(106, 108, 118), grid_color=(140, 142, 150),
        boxes=boxes, flows=flows, ceiling_z=8.0, ceiling_color=(60, 64, 72),
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
    wind_dir = (3.6, 0.8, 0.0)
    flows = [
        FlowSpec("wind", {"dir": wind_dir, "gust": 0.55, "period": 7.0, "turbulence": 0.7, "shear_ref": 3.0}),
    ]
    cloths = [ClothSpec(anchor=(0.12, 0.0, 3.0), width=2.8, height=1.5, color=(220, 80, 70), label="windsock-flag", kind="flag", plane="xz")]
    rigids = [
        RigidSpec("crate", (185, 150, 90), (0.7, 0.7, 0.7), mass=0.4, pos=(-5.0, 1.0, 0.4), spin=(0.3, 0.0, 0.4)),
        RigidSpec("debris", (170, 165, 150), (0.5, 0.5, 0.3), mass=0.15, pos=(-3.0, -2.0, 0.3)),
    ]
    particles = [ParticleSpec("dust", (-6.0, 0.0, 0.2), count=70, color=(206, 196, 170), spawn_radius=9.0, lifetime=5.0)]
    return Scene(
        id="open_field", name="Open field (outdoor)", kind="outdoor",
        sky_top=(64, 104, 162), sky_bottom=(160, 188, 212),
        ground_color=(78, 118, 70), ground_alt=(70, 108, 62), grid_color=(96, 132, 84),
        boxes=boxes, flows=flows, cloths=cloths, rigids=rigids, particles=particles, far=60.0, fog_density=0.012,
    )


def _clothing_store() -> Scene:
    """Indoor retail floor: garment racks with hanging cloth, mannequins, AC
    draft. Designed to show off the high-fidelity (PyBullet) physics backend —
    the garments are real deformable cloth that sways in the airflow."""
    boxes: list[Box] = []
    garment_palette = [(196, 86, 96), (78, 120, 180), (84, 154, 110), (210, 178, 86), (150, 110, 180), (90, 96, 110),
                       (220, 140, 80), (60, 150, 160), (200, 90, 150), (130, 130, 140)]

    # Garment racks: full shop floor. CLOTH INSTANCING — three master panels are
    # simulated as soft bodies; every garment on every rail is a rendered instance
    # of a master with its own sway. Hundreds of garments, three sims.
    rack_anchors_a: list[tuple] = []  # master 0 (shirts, plane xz)
    rack_anchors_b: list[tuple] = []  # master 1 (dresses, longer)
    rack_anchors_c: list[tuple] = []  # master 2 (jackets, plane yz on side racks)
    colors_a: list = []
    colors_b: list = []
    colors_c: list = []
    rail_z = 1.65
    # A large spread-out shop floor: 12 racks × 10 garments = 120 deformable
    # garments, all instanced from three simulated masters.
    racks = [(rx, ry) for rx in (-9.0, -3.0, 3.0, 9.0) for ry in (-7.0, 0.0, 7.0)]
    for ri, (rx, ry) in enumerate(racks):
        boxes.append(Box((rx - 1.4, ry, rail_z / 2), (0.1, 0.1, rail_z), (150, 150, 158), "rack-post"))
        boxes.append(Box((rx + 1.4, ry, rail_z / 2), (0.1, 0.1, rail_z), (150, 150, 158), "rack-post"))
        boxes.append(Box((rx, ry, rail_z), (2.9, 0.1, 0.08), (170, 172, 180), "rack-rail"))
        for gi in range(10):  # 10 garments per rail
            gx = rx - 1.25 + gi * (2.5 / 9)
            anchor = (gx, ry, rail_z)
            color = garment_palette[(ri * 10 + gi) % len(garment_palette)]
            kindsel = (ri + gi) % 3
            if kindsel == 0:
                rack_anchors_a.append(anchor); colors_a.append(color)
            elif kindsel == 1:
                rack_anchors_b.append(anchor); colors_b.append(color)
            else:
                rack_anchors_c.append(anchor); colors_c.append(color)

    cloth_groups = [
        ClothInstanceGroup(
            master=ClothSpec(anchor=(-9.0, -7.0, rail_z), color=(196, 86, 96), width=0.5, height=0.9,
                             mass=0.08, label="__master__0", kind="garment", plane="xz"),
            anchors=rack_anchors_a, colors=colors_a, label="shirt"),
        ClothInstanceGroup(
            master=ClothSpec(anchor=(9.0, -7.0, rail_z), color=(78, 120, 180), width=0.55, height=1.15,
                             mass=0.1, label="__master__1", kind="garment", plane="xz"),
            anchors=rack_anchors_b, colors=colors_b, label="dress"),
        ClothInstanceGroup(
            master=ClothSpec(anchor=(3.0, -7.0, rail_z), color=(120, 110, 100), width=0.6, height=0.95,
                             mass=0.1, label="__master__2", kind="garment", plane="xz"),
            anchors=rack_anchors_c, colors=colors_c, label="jacket"),
    ]

    # Wall shelves with folded stock, mannequins, a checkout counter, fitting rooms.
    for sx in (-10.5, 10.5):
        for sz in (0.8, 1.6, 2.4):
            boxes.append(Box((sx, 0, sz), (0.5, 14, 0.12), (160, 150, 138), "shelf"))
    for k, (mx, my) in enumerate([(-9, -7), (-9, 7), (9, -7), (9, 7), (0, -9)]):
        boxes.append(Box((mx, my, 0.95), (0.5, 0.3, 1.9), _jit((205, 198, 188), k, 12), "mannequin"))
    boxes.append(Box((0, 9, 0.55), (5.0, 1.4, 1.1), (120, 110, 96), "counter"))
    for fx in (-10.5, -8.0, 8.0, 10.5):
        boxes.append(Box((fx, 10.5, 1.2), (0.12, 2.0, 2.4), (130, 134, 142), "fitting-wall"))
    # Perimeter walls.
    boxes.append(Box((0, -11.5, 1.4), (24, 0.3, 2.8), (172, 168, 162), "wall"))
    boxes.append(Box((0, 11.5, 1.4), (24, 0.3, 2.8), (172, 168, 162), "wall"))

    # A toppled stock crate + a shopping basket as free rigid bodies.
    rigids = [
        RigidSpec("crate", (180, 140, 90), (0.6, 0.6, 0.6), mass=0.5, pos=(3.0, -9.0, 0.35)),
        RigidSpec("basket", (160, 70, 70), (0.45, 0.6, 0.4), mass=0.2, pos=(-3.0, 9.0, 0.3), spin=(0.0, 0.0, 0.2)),
    ]
    # Ceiling AC vents push air across the racks so the garments sway + ripple.
    flows = [
        FlowSpec("wind", {"dir": (0.8, 0.0, 0.0), "gust": 0.5, "period": 6.0, "turbulence": 0.4, "shear_ref": 2.5}),
        FlowSpec("fan", {"pos": (-11.0, 0.0, 2.9), "dir": (1.0, 0.0, -0.15), "speed": 5.5, "radius": 1.4, "reach": 18.0, "spread": 0.7}, label="ac-vent"),
    ]
    return Scene(
        id="clothing_store", name="Clothing store (indoor)", kind="indoor",
        sky_top=(96, 98, 110), sky_bottom=(150, 150, 162),
        ground_color=(150, 142, 132), ground_alt=(140, 132, 122), grid_color=(168, 160, 150),
        boxes=boxes, flows=flows, rigids=rigids, cloth_groups=cloth_groups,
        ceiling_z=3.2, ceiling_color=(196, 196, 204),
    )


def _retail_store() -> Scene:
    """General retail floor: stocked aisles, end-caps, checkout, shopping carts
    (free rigid bodies), promo banners (cloth), AC airflow."""
    boxes: list[Box] = []
    cloths: list[ClothSpec] = []
    product_palette = [(196, 86, 96), (78, 120, 180), (84, 154, 110), (210, 178, 86), (150, 110, 180), (200, 130, 70)]

    # Gondola aisles: long double-sided shelving with stocked product blocks.
    for ai, ax in enumerate((-7.5, -2.5, 2.5, 7.5)):
        boxes.append(Box((ax, 0, 1.0), (1.6, 13.0, 2.0), (150, 152, 158), "gondola"))
        for level in range(3):
            for j, y in enumerate(range(-6, 7, 2)):
                color = product_palette[(ai + level + j) % len(product_palette)]
                boxes.append(Box((ax, y, 0.5 + level * 0.7), (1.7, 1.6, 0.5), _jit(color, ai + j + level, 14), "product"))
        # End-cap displays.
        boxes.append(Box((ax, 7.2, 0.7), (1.6, 1.0, 1.4), _jit((200, 120, 70), ai), "endcap"))

    # Checkout lanes + entrance, perimeter walls (collision), promo banners.
    for lx in (-9.5, -8.0, 8.0, 9.5):
        boxes.append(Box((lx, -9.5, 0.5), (1.0, 2.4, 1.0), (120, 124, 132), "checkout"))
    boxes.append(Box((0, -11.6, 1.4), (24, 0.3, 2.8), (168, 168, 172), "wall"))
    boxes.append(Box((0, 11.6, 1.4), (24, 0.3, 2.8), (168, 168, 172), "wall"))
    boxes.append(Box((-12, 0, 1.4), (0.3, 24, 2.8), (168, 168, 172), "wall"))
    boxes.append(Box((12, 0, 1.4), (0.3, 24, 2.8), (168, 168, 172), "wall"))
    for bx in (-5.0, 0.0, 5.0):
        cloths.append(ClothSpec(anchor=(bx, 9.5, 3.0), width=1.6, height=0.9, color=_jit((210, 80, 80), int(bx + 6)),
                                label="promo-banner", kind="banner", plane="xz"))

    # Shopping carts + a knocked-over stock box the AC nudges.
    rigids = [
        RigidSpec("cart", (180, 184, 190), (0.6, 0.9, 0.7), mass=0.5, pos=(-5.0, -8.0, 0.4), spin=(0.0, 0.0, 0.1)),
        RigidSpec("cart", (180, 184, 190), (0.6, 0.9, 0.7), mass=0.5, pos=(5.0, -7.0, 0.4)),
        RigidSpec("stock-box", (175, 140, 85), (0.5, 0.5, 0.5), mass=0.25, pos=(0.0, 5.0, 0.3)),
    ]
    flows = [
        FlowSpec("wind", {"dir": (0.0, 0.9, 0.0), "gust": 0.4, "period": 7.0, "turbulence": 0.3, "shear_ref": 2.5}),
        FlowSpec("fan", {"pos": (0.0, -11.0, 2.9), "dir": (0.0, 1.0, -0.1), "speed": 5.0, "radius": 1.6, "reach": 20.0, "spread": 0.8}, label="ac-vent"),
    ]
    return Scene(
        id="retail_store", name="Retail store (indoor)", kind="indoor",
        sky_top=(110, 112, 122), sky_bottom=(158, 158, 168),
        ground_color=(158, 152, 144), ground_alt=(148, 142, 134), grid_color=(176, 170, 160),
        boxes=boxes, flows=flows, rigids=rigids, cloths=cloths,
        ceiling_z=3.2, ceiling_color=(208, 208, 214),
    )


def _house_on_fire() -> Scene:
    """A house interior with rooms (walls = collision geometry), furniture, an
    active fire with heavy volumetric smoke, heat updrafts, falling debris and
    a burning curtain. The firefighter-rescue search scenario."""
    boxes: list[Box] = []

    # Outer shell + internal partitions forming four rooms with doorway gaps.
    boxes.append(Box((0, -9, 1.4), (20, 0.3, 2.8), (140, 120, 110), "wall"))
    boxes.append(Box((0, 9, 1.4), (20, 0.3, 2.8), (140, 120, 110), "wall"))
    boxes.append(Box((-10, 0, 1.4), (0.3, 18, 2.8), (140, 120, 110), "wall"))
    boxes.append(Box((10, 0, 1.4), (0.3, 18, 2.8), (140, 120, 110), "wall"))
    # Internal cross-walls (leave doorway gaps by splitting into segments).
    for seg in ((-7.5, -3.5), (3.5, 7.5)):  # horizontal partition on y=0 with a central door
        cx = (seg[0] + seg[1]) / 2
        boxes.append(Box((cx, 0, 1.4), (seg[1] - seg[0], 0.3, 2.8), (134, 116, 106), "wall"))
    for seg in ((-7.0, -2.5), (2.5, 7.0)):  # vertical partition on x=0
        cy = (seg[0] + seg[1]) / 2
        boxes.append(Box((0, cy, 1.4), (0.3, seg[1] - seg[0], 2.8), (134, 116, 106), "wall"))

    # Furniture (collision + clutter): sofa, table, beds, shelves, fridge.
    boxes.append(Box((-6, -5, 0.45), (3.0, 1.2, 0.9), (90, 100, 130), "sofa"))
    boxes.append(Box((-5, 4.5, 0.4), (2.0, 1.2, 0.8), (150, 110, 70), "table"))
    boxes.append(Box((6, -5, 0.45), (2.4, 1.6, 0.9), (160, 150, 170), "bed"))
    boxes.append(Box((6, 5, 0.45), (2.4, 1.6, 0.9), (170, 160, 150), "bed"))
    boxes.append(Box((8.6, 0, 1.0), (0.6, 2.0, 2.0), (190, 192, 198), "fridge"))
    boxes.append(Box((-9, 6, 1.0), (0.5, 3.0, 2.0), (140, 110, 80), "shelf"))

    # The fire: seated at the sofa, throwing a tall smoke column; a second
    # smouldering source by the table fills the adjoining room with haze.
    fires = [
        FireSpec((-6.0, -5.0, 0.6), radius=0.8, height=2.2, intensity=1.3,
                 smoke_rate=90, flame_rate=40, rise=4.0, smoke_color=(46, 44, 46), label="sofa-fire"),
        FireSpec((-5.0, 4.5, 0.5), radius=0.5, height=1.2, intensity=0.7,
                 smoke_rate=60, flame_rate=22, rise=3.0, smoke_color=(54, 52, 54), label="table-fire"),
    ]
    # Extra drifting smoke pooling under the ceiling (search-and-rescue haze).
    smokes = [
        SmokeSpec((-6.0, -5.0, 2.4), radius=1.4, color=(60, 58, 60), rate=70, rise=0.6, lifetime=8.0, density=0.6, end_radius=3.4),
        SmokeSpec((4.0, 4.0, 2.2), radius=1.6, color=(72, 70, 72), rate=50, rise=0.5, lifetime=8.0, density=0.5, end_radius=3.0),
    ]
    # Heat updrafts over each fire (buoyant plume drives the drones too).
    flows = [
        FlowSpec("updraft", {"cx": -6.0, "cy": -5.0, "radius": 3.0, "speed": 3.5, "top": 3.0}),
        FlowSpec("updraft", {"cx": -5.0, "cy": 4.5, "radius": 2.2, "speed": 2.2, "top": 3.0}),
        FlowSpec("wind", {"dir": (0.6, 0.3, 0.0), "gust": 0.5, "period": 5.0, "turbulence": 0.6, "shear_ref": 2.5}),
    ]
    # Falling ceiling debris (free rigid bodies dropping into the rooms).
    rigids = [
        RigidSpec("debris", (110, 96, 86), (0.5, 0.5, 0.3), mass=0.3, pos=(-6.0, -5.0, 2.6)),
        RigidSpec("debris", (120, 104, 92), (0.4, 0.6, 0.3), mass=0.25, pos=(-4.0, 4.0, 2.6), spin=(0.5, 0.0, 0.3)),
        RigidSpec("debris", (100, 90, 84), (0.6, 0.4, 0.35), mass=0.35, pos=(5.0, -4.5, 2.6)),
    ]
    # A burning curtain (deformable cloth) by the window.
    cloths = [
        ClothSpec(anchor=(-9.5, -6.0, 2.6), width=1.2, height=2.0, color=(180, 90, 60), label="curtain", kind="curtain", plane="yz"),
        ClothSpec(anchor=(9.6, 3.0, 2.6), width=1.2, height=2.0, color=(150, 140, 150), label="curtain", kind="curtain", plane="yz"),
    ]
    return Scene(
        id="house_on_fire", name="House on fire (indoor)", kind="indoor",
        sky_top=(40, 30, 28), sky_bottom=(70, 54, 46),
        ground_color=(96, 84, 78), ground_alt=(86, 76, 70), grid_color=(120, 96, 80),
        boxes=boxes, flows=flows, rigids=rigids, cloths=cloths, fires=fires, smokes=smokes,
        ceiling_z=2.9, ceiling_color=(54, 44, 42), fog_density=0.05,
    )


_BUILDERS = {
    "open_field": _open_field,
    "warehouse": _warehouse,
    "office": _office,
    "city": _city,
    "park": _park,
    "construction": _construction,
    "atrium": _atrium,
    "clothing_store": _clothing_store,
    "retail_store": _retail_store,
    "house_on_fire": _house_on_fire,
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


def dynamic_objects(scene: Scene, t: float) -> list[Box]:
    """Current-position boxes for the scene's moving objects at time ``t`` (s)."""
    out: list[Box] = []
    for spec in scene.dynamics:
        out.append(Box(spec.position_at(t), spec.size, spec.color, spec.label))
    return out
