"""
High-fidelity physics backend for the swarm sim — the *environment* half.

Where :mod:`drone_control.sim.rigid` is a hand-rolled, fully-numpy rigid engine,
this module drives the same scene through **PyBullet** when a session asks for
the high-fidelity backend. It owns ENVIRONMENT bodies only — free rigid boxes
(crates, pallets, debris) and deformable cloth (flags, banners, curtains) — and
makes them feel the very wind the drones feel, so a gust streams a banner and
shoves a crate while the drones (simulated elsewhere) fly through it.

Everything runs fully headless (``p.connect(p.DIRECT)``) and deterministic: a
fixed timestep, sub-stepped soft-body solve, and no wall-clock or unseeded RNG.
Each :class:`PyBulletWorld` owns its own physics client id and threads it through
every PyBullet call, so several worlds can coexist without colliding.

World frame matches the rest of the sim: z up, ground plane at ``z=0``.

Rigid aero matches :mod:`rigid` and :mod:`flow` exactly::

    F = c_aero * |v_rel| * v_rel,   v_rel = wind - body velocity,
    c_aero = 0.5 * rho * Cd * A.

Cloth wind — chosen approach (documented):
    PyBullet's per-body / per-node ``applyExternalForce`` has *no* effect on a
    soft body in this build (tested: the mesh does not move), so we cannot push
    cloth that way. Instead we drive the cloth with a per-step **gravity bias**:
    each step we set the deformable-world gravity to ``(-down) + k * wind_mean``,
    where ``wind_mean`` is the mean wind sampled at the cloth's current vertices
    and ``k`` is a small aero gain. This is the same body acceleration a uniform
    wind would impart, applied through the one channel the soft-body solver
    honours, so the panel visibly streams/deflects under a steady wind while its
    pinned top edge holds. The bias is clamped and the solver is sub-stepped with
    extra iterations so the mass-spring panel stays bounded and finite.
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np

try:  # PyBullet is only needed for the high-fidelity backend.
    import pybullet as p
    import pybullet_data
except Exception as exc:  # pragma: no cover - import guard
    p = None  # type: ignore[assignment]
    pybullet_data = None  # type: ignore[assignment]
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None

from drone_control.sim.flow import AIR_DENSITY
from drone_control.sim.scenes import ClothSpec, RigidSpec

# Stability clamps (mirror rigid.py's spirit).
_MAX_VEL = 30.0       # m/s, used to sanitize sampled wind
_MAX_FORCE = 5.0e3    # N, per-body external force clamp
_MAX_POS = 1.0e4      # m, absurd-position guard

# Soft-body solver settings. The bundled ``cloth_z_up.obj`` (~25 verts) with a
# stiff mass-spring is unstable at dt=0.02 with a single explicit step, so we
# sub-step and add solver iterations; these values keep a hanging panel bounded
# while staying responsive to the wind bias (tuned empirically, see module doc).
_NUM_SOLVER_ITERS = 40
_NUM_SUBSTEPS = 4
_CLOTH_DAMPING = 0.3          # springDampingStiffness
_CLOTH_FRICTION = 0.5
_CLOTH_WIND_GAIN = 0.8        # wind (m/s) -> gravity-bias accel; see step()
_CLOTH_BIAS_CLAMP = 40.0      # max horizontal bias accel (m/s^2)

# Per-face shading + cube corner ordering, identical to rigid.py so the renderer
# paints a PyBullet box exactly like a hand-rolled one.
_FACE_AXES = [
    (2, +1),  # top
    (2, -1),  # bottom
    (0, +1),  # +x side
    (0, -1),  # -x side
    (1, +1),  # +y side
    (1, -1),  # -y side
]
_FACE_CORNERS = {
    (2, +1): [(-1, -1, +1), (+1, -1, +1), (+1, +1, +1), (-1, +1, +1)],
    (2, -1): [(-1, -1, -1), (-1, +1, -1), (+1, +1, -1), (+1, -1, -1)],
    (0, +1): [(+1, -1, -1), (+1, +1, -1), (+1, +1, +1), (+1, -1, +1)],
    (0, -1): [(-1, -1, -1), (-1, -1, +1), (-1, +1, +1), (-1, +1, -1)],
    (1, +1): [(-1, +1, -1), (-1, +1, +1), (+1, +1, +1), (+1, +1, -1)],
    (1, -1): [(-1, -1, -1), (+1, -1, -1), (+1, -1, +1), (-1, -1, +1)],
}


# The bundled cloth mesh's triangle topology, parsed once from the .obj. The
# soft-body simulation mesh keeps the same vertex ordering as the source file,
# so these face index triples stay valid as the cloth deforms. PyBullet's
# ``getMeshData`` returns only ``(nverts, verts)`` in this build (no index
# buffer), so we recover the faces from the asset itself.
_CLOTH_MESH_NAME = "cloth_z_up.obj"
_cloth_tris_cache: list[tuple[int, int, int]] | None = None


def _cloth_mesh_triangles() -> list[tuple[int, int, int]]:
    """Triangle index triples for ``cloth_z_up.obj`` (cached, fan-split quads)."""
    global _cloth_tris_cache
    if _cloth_tris_cache is not None:
        return _cloth_tris_cache
    tris: list[tuple[int, int, int]] = []
    try:
        path = os.path.join(pybullet_data.getDataPath(), _CLOTH_MESH_NAME)
        with open(path) as fh:
            for line in fh:
                if not line.startswith("f "):
                    continue
                idx = [int(tok.split("/")[0]) - 1 for tok in line.split()[1:]]
                if len(idx) == 3:
                    tris.append((idx[0], idx[1], idx[2]))
                elif len(idx) >= 4:  # fan-triangulate polygons
                    for k in range(1, len(idx) - 1):
                        tris.append((idx[0], idx[k], idx[k + 1]))
    except Exception as exc:  # pragma: no cover - degrade to no faces
        print(f"[physics] cloth topology parse failed: {exc}", file=sys.stderr)
        tris = []
    _cloth_tris_cache = tris
    return tris


def _quat_xyzw_to_rotmat(q) -> np.ndarray:
    """PyBullet (x,y,z,w) quaternion -> 3x3 rotation matrix."""
    x, y, z, w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


class PyBulletWorld:
    """PyBullet-backed environment: free rigid boxes + deformable cloth.

    The drones are simulated elsewhere; this world owns scenery the airflow
    blows around. ``step`` is driven by the same ``wind_at`` callback the rigid
    and flow modules use, so disturbances are consistent across backends.
    """

    def __init__(
        self,
        rigids: list[RigidSpec],
        cloths: list[ClothSpec],
        *,
        dt: float = 0.02,
        gravity: float = 9.81,
        ground_z: float = 0.0,
        seed: int = 0,
    ) -> None:
        if p is None:  # pragma: no cover - guarded import
            raise RuntimeError(f"pybullet is unavailable: {_IMPORT_ERROR}")

        self._rng = np.random.default_rng(seed)
        self.dt = float(dt)
        self.gravity = float(gravity)
        self.ground_z = float(ground_z)
        self._closed = False

        self._has_cloth = bool(cloths)
        # DIRECT (headless) client, one per world; threaded through every call.
        self.cid = p.connect(p.DIRECT)

        if self._has_cloth:
            # Deformable world is required for soft bodies; resetSimulation with
            # the flag wipes the default world, so do it before anything else.
            p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD, physicsClientId=self.cid)

        if pybullet_data is not None:
            p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.cid)

        p.setGravity(0.0, 0.0, -self.gravity, physicsClientId=self.cid)
        p.setPhysicsEngineParameter(
            fixedTimeStep=self.dt,
            numSolverIterations=_NUM_SOLVER_ITERS,
            numSubSteps=_NUM_SUBSTEPS,
            physicsClientId=self.cid,
        )

        self._add_ground()

        # --- rigid bodies -------------------------------------------------
        self.body_ids: list[int] = []
        self.colors: list[tuple[int, int, int]] = []
        self.labels: list[str] = []
        self.sizes: list[np.ndarray] = []
        self.c_aero: list[float] = []
        for spec in rigids:
            self._add_rigid(spec)

        # --- cloth bodies -------------------------------------------------
        self.cloth_ids: list[int] = []
        self.cloth_colors: list[tuple[int, int, int]] = []
        self.cloth_labels: list[str] = []
        self.cloth_areas: list[float] = []
        for spec in cloths:
            self._add_cloth(spec)

    # ------------------------------------------------------------------ build

    def _add_ground(self) -> None:
        """Static ground plane at ``ground_z`` (a large thin box, robust + cheap)."""
        try:
            half = [200.0, 200.0, 0.5]
            col = p.createCollisionShape(
                p.GEOM_BOX, halfExtents=half, physicsClientId=self.cid
            )
            vis = p.createVisualShape(
                p.GEOM_BOX, halfExtents=half, rgbaColor=[0.5, 0.5, 0.5, 1.0],
                physicsClientId=self.cid,
            )
            gid = p.createMultiBody(
                baseMass=0.0,
                baseCollisionShapeIndex=col,
                baseVisualShapeIndex=vis,
                basePosition=[0.0, 0.0, self.ground_z - 0.5],
                physicsClientId=self.cid,
            )
            p.changeDynamics(
                gid, -1, restitution=0.3, lateralFriction=0.8, physicsClientId=self.cid
            )
        except Exception as exc:  # pragma: no cover - degrade, never crash
            print(f"[physics] ground plane failed: {exc}", file=sys.stderr)

    def _add_rigid(self, spec: RigidSpec) -> None:
        """One free rigid box: collision+visual GEOM_BOX with mass, vel, spin."""
        try:
            size = np.asarray(spec.size, dtype=np.float64)
            half = (size * 0.5).tolist()
            mass = max(1e-6, float(spec.mass))
            rgba = [c / 255.0 for c in spec.color] + [1.0]
            col = p.createCollisionShape(
                p.GEOM_BOX, halfExtents=half, physicsClientId=self.cid
            )
            vis = p.createVisualShape(
                p.GEOM_BOX, halfExtents=half, rgbaColor=rgba, physicsClientId=self.cid
            )
            bid = p.createMultiBody(
                baseMass=mass,
                baseCollisionShapeIndex=col,
                baseVisualShapeIndex=vis,
                basePosition=list(spec.pos),
                physicsClientId=self.cid,
            )
            p.resetBaseVelocity(
                bid,
                linearVelocity=list(spec.vel),
                angularVelocity=list(spec.spin),
                physicsClientId=self.cid,
            )
            p.changeDynamics(
                bid,
                -1,
                restitution=float(np.clip(spec.restitution, 0.0, 1.0)),
                lateralFriction=float(np.clip(spec.friction, 0.0, 1.0)),
                physicsClientId=self.cid,
            )
            # c_aero = 0.5 * rho * Cd * A, A = mean of the three face areas.
            sx, sy, sz = size
            area = (sy * sz + sx * sz + sx * sy) / 3.0
            c_aero = 0.5 * AIR_DENSITY * float(spec.drag_cd) * float(area)

            self.body_ids.append(bid)
            self.colors.append(tuple(int(c) for c in spec.color))
            self.labels.append(spec.label)
            self.sizes.append(size)
            self.c_aero.append(c_aero)
        except Exception as exc:  # pragma: no cover - degrade, never crash
            print(f"[physics] rigid '{spec.label}' failed: {exc}", file=sys.stderr)

    def _add_cloth(self, spec: ClothSpec) -> None:
        """Deformable mass-spring panel pinned along its top edge to ``anchor``.

        The bundled ``cloth_z_up.obj`` is a unit plane lying flat in xy. We rotate
        it to stand vertically (``plane='xz'`` faces +-y, ``plane='yz'`` faces
        +-x), scale it to roughly ``width x height``, and place it so its top edge
        sits at ``anchor``; then we anchor every top-edge node to the world.
        """
        try:
            if pybullet_data is None:
                raise RuntimeError("pybullet_data unavailable for cloth mesh")

            anchor = np.asarray(spec.anchor, dtype=np.float64)
            plane = (spec.plane or "xz").lower()
            # The unit plane spans roughly [-1,1] in x and y. We orient and scale
            # it so it hangs down from the anchor. A single uniform scale keeps the
            # mesh well-conditioned; width/height are matched on average.
            scale = max(0.05, 0.5 * (float(spec.width) + float(spec.height)))

            if plane == "yz":
                # Stand the plane up facing +-x: rotate about +y so it lies in yz.
                orn = p.getQuaternionFromEuler([0.0, math.pi / 2.0, 0.0])
            else:  # "xz" (default): stand it up facing +-y.
                orn = p.getQuaternionFromEuler([-math.pi / 2.0, 0.0, 0.0])

            # Drop the load point up by ~scale so the (rotated) panel's *top* edge
            # ends up near the anchor; the exact top nodes are pinned below.
            base_pos = [float(anchor[0]), float(anchor[1]), float(anchor[2])]

            cloth_id = p.loadSoftBody(
                "cloth_z_up.obj",
                basePosition=base_pos,
                baseOrientation=orn,
                scale=scale,
                mass=max(1e-3, float(spec.mass)),
                useNeoHookean=0,
                useBendingSprings=1,
                useMassSpring=1,
                springElasticStiffness=float(spec.stiffness),
                springDampingStiffness=_CLOTH_DAMPING,
                frictionCoeff=_CLOTH_FRICTION,
                useSelfCollision=0,
                physicsClientId=self.cid,
            )

            # Pin the top edge: read the simulation mesh, find max-z vertices.
            nverts, verts = p.getMeshData(
                cloth_id, flags=p.MESH_DATA_SIMULATION_MESH, physicsClientId=self.cid
            )
            v = np.asarray(verts, dtype=np.float64).reshape(-1, 3)
            if v.shape[0] == 0:
                raise RuntimeError("cloth mesh has no vertices")
            zmax = float(v[:, 2].max())
            tol = 1e-3 * max(1.0, scale)
            top_idx = np.where(v[:, 2] >= zmax - tol)[0]
            if top_idx.size == 0:
                top_idx = np.array([int(np.argmax(v[:, 2]))])
            for i in top_idx:
                p.createSoftBodyAnchor(cloth_id, int(i), -1, -1, physicsClientId=self.cid)

            # Panel area for the wind aero gain (use the spec's nominal size).
            area = max(1e-3, float(spec.width) * float(spec.height))

            self.cloth_ids.append(cloth_id)
            self.cloth_colors.append(tuple(int(c) for c in spec.color))
            self.cloth_labels.append(spec.label)
            self.cloth_areas.append(area)
        except Exception as exc:  # degrade gracefully, never crash
            print(f"[physics] cloth '{spec.label}' failed, skipping: {exc}", file=sys.stderr)

    # ------------------------------------------------------------------- step

    def step(self, dt: float, wind_at) -> None:
        """Advance one fixed step. ``wind_at(points[N,3]) -> wind[N,3]`` (m/s)."""
        if self._closed:
            return
        try:
            # --- rigid aero: F = c_aero * |v_rel| * v_rel at each body center ---
            if self.body_ids:
                centers = np.array(
                    [
                        p.getBasePositionAndOrientation(b, physicsClientId=self.cid)[0]
                        for b in self.body_ids
                    ],
                    dtype=np.float64,
                )
                linvels = np.array(
                    [
                        p.getBaseVelocity(b, physicsClientId=self.cid)[0]
                        for b in self.body_ids
                    ],
                    dtype=np.float64,
                )
                w = np.atleast_2d(np.asarray(wind_at(centers), dtype=np.float64))
                w = np.nan_to_num(w, nan=0.0, posinf=_MAX_VEL, neginf=-_MAX_VEL)
                v_rel = w - linvels
                speed = np.linalg.norm(v_rel, axis=1, keepdims=True)
                c = np.asarray(self.c_aero, dtype=np.float64)[:, None]
                forces = c * speed * v_rel
                forces = np.clip(
                    np.nan_to_num(forces, nan=0.0), -_MAX_FORCE, _MAX_FORCE
                )
                for bid, force, pos in zip(self.body_ids, forces, centers):
                    p.applyExternalForce(
                        bid,
                        -1,
                        force.tolist(),
                        pos.tolist(),
                        p.WORLD_FRAME,
                        physicsClientId=self.cid,
                    )

            # --- cloth wind via gravity bias (see module docstring) ------------
            # PyBullet ignores per-body force on soft bodies in this build, so we
            # encode a uniform wind acceleration as a per-step gravity bias. We
            # use the mean wind over each cloth's *current* vertices, scaled by a
            # small aero gain, and apply it for that cloth's solve. With one
            # deformable world we apply the mean over all cloths (they share the
            # same global wind field in practice); the down component is fixed.
            if self.cloth_ids:
                means = []
                for cloth_id in self.cloth_ids:
                    pts = self._cloth_vertices(cloth_id)
                    if pts.shape[0] == 0:
                        continue
                    cw = np.atleast_2d(np.asarray(wind_at(pts), dtype=np.float64))
                    cw = np.nan_to_num(cw, nan=0.0, posinf=_MAX_VEL, neginf=-_MAX_VEL)
                    means.append(cw.mean(axis=0))
                if means:
                    wind_mean = np.mean(means, axis=0)
                    bias = np.clip(
                        _CLOTH_WIND_GAIN * wind_mean,
                        -_CLOTH_BIAS_CLAMP,
                        _CLOTH_BIAS_CLAMP,
                    )
                    p.setGravity(
                        float(bias[0]),
                        float(bias[1]),
                        -self.gravity,
                        physicsClientId=self.cid,
                    )
                else:
                    p.setGravity(0.0, 0.0, -self.gravity, physicsClientId=self.cid)

            p.stepSimulation(physicsClientId=self.cid)

            # Restore plain gravity so a step with no wind sample is unbiased.
            if self.cloth_ids:
                p.setGravity(0.0, 0.0, -self.gravity, physicsClientId=self.cid)
        except Exception as exc:  # log and continue; never crash the sim loop
            print(f"[physics] step error: {exc}", file=sys.stderr)

    def _cloth_vertices(self, cloth_id: int) -> np.ndarray:
        """Current simulation-mesh vertices of a cloth as float64 [V,3]."""
        try:
            _, verts = p.getMeshData(
                cloth_id, flags=p.MESH_DATA_SIMULATION_MESH, physicsClientId=self.cid
            )
            v = np.asarray(verts, dtype=np.float64).reshape(-1, 3)
            return np.nan_to_num(v, nan=0.0, posinf=_MAX_POS, neginf=-_MAX_POS)
        except Exception:
            return np.zeros((0, 3), dtype=np.float64)

    # ----------------------------------------------------------------- render

    def rigid_faces(self) -> list[tuple[list[np.ndarray], tuple[int, int, int], str]]:
        """Six shaded quads per rigid body, as ``(corners, color, label)``.

        Same shape as :meth:`rigid.RigidWorld.face_quads` so the renderer paints
        a PyBullet box identically to a hand-rolled one.
        """
        out: list[tuple[list[np.ndarray], tuple[int, int, int], str]] = []
        if self._closed or not self.body_ids:
            return out
        for bid, color, label, size in zip(
            self.body_ids, self.colors, self.labels, self.sizes
        ):
            try:
                pos, orn = p.getBasePositionAndOrientation(bid, physicsClientId=self.cid)
                center = np.asarray(pos, dtype=np.float64)
                R = _quat_xyzw_to_rotmat(orn)
                half = np.asarray(size, dtype=np.float64) * 0.5
                base = np.asarray(color, dtype=np.float64)
                for axis, sign in _FACE_AXES:
                    normal_body = np.zeros(3)
                    normal_body[axis] = float(sign)
                    nz = float((R @ normal_body)[2])
                    if nz > 0.5:
                        shade = 1.0
                    elif nz < -0.5:
                        shade = 0.5
                    else:
                        shade = 0.8 if axis == 0 else 0.68
                    fcolor = tuple(int(c) for c in np.clip(base * shade, 0, 255))
                    corners = []
                    for sgn in _FACE_CORNERS[(axis, sign)]:
                        local = np.asarray(sgn, dtype=np.float64) * half
                        corners.append(center + R @ local)
                    out.append((corners, fcolor, label))
            except Exception as exc:  # pragma: no cover - skip a bad body
                print(f"[physics] rigid_faces '{label}' failed: {exc}", file=sys.stderr)
        return out

    def cloth_meshes(
        self,
    ) -> list[tuple[np.ndarray, list[tuple[int, int, int]], tuple[int, int, int], str]]:
        """Per cloth: ``(vertices[V,3], triangle_faces, color, label)``."""
        out: list[
            tuple[np.ndarray, list[tuple[int, int, int]], tuple[int, int, int], str]
        ] = []
        if self._closed or not self.cloth_ids:
            return out
        for cloth_id, color, label in zip(
            self.cloth_ids, self.cloth_colors, self.cloth_labels
        ):
            try:
                nverts, verts = p.getMeshData(
                    cloth_id, flags=p.MESH_DATA_SIMULATION_MESH, physicsClientId=self.cid
                )
                v = np.asarray(verts, dtype=np.float64).reshape(-1, 3)
                v = np.nan_to_num(v, nan=0.0, posinf=_MAX_POS, neginf=-_MAX_POS)
                tris = self._cloth_triangles(cloth_id, v.shape[0])
                out.append((v, tris, color, label))
            except Exception as exc:  # pragma: no cover - skip a bad cloth
                print(f"[physics] cloth_meshes '{label}' failed: {exc}", file=sys.stderr)
        return out

    def _cloth_triangles(self, cloth_id: int, nverts: int) -> list[tuple[int, int, int]]:
        """Triangle index triples for a cloth's simulation mesh.

        ``getMeshData`` returns no index buffer in this build, but the soft body
        preserves the source mesh's vertex ordering, so we use the topology
        parsed from ``cloth_z_up.obj`` (cached). Defensive: drop any face that
        references a vertex beyond the current mesh.
        """
        # If the index buffer ever appears as a third element, prefer it.
        try:
            data = p.getMeshData(
                cloth_id, flags=p.MESH_DATA_SIMULATION_MESH, physicsClientId=self.cid
            )
            if len(data) >= 3 and data[2]:
                flat = list(data[2])
                tris = []
                for k in range(0, len(flat) - 2, 3):
                    i, j, l = int(flat[k]), int(flat[k + 1]), int(flat[k + 2])
                    if max(i, j, l) < nverts:
                        tris.append((i, j, l))
                if tris:
                    return tris
        except Exception:
            pass
        return [t for t in _cloth_mesh_triangles() if max(t) < nverts]

    # ------------------------------------------------------------- ground truth

    def bodies_gt(self) -> list[tuple[str, list[float], list[float]]]:
        """Per-body ground truth: ``(label, center[xyz], aabb_size[sx,sy,sz])``.

        Covers rigid bodies (via PyBullet's AABB) and cloths (vertex bounds);
        feeds the sim-privileged depth/landmark path.
        """
        out: list[tuple[str, list[float], list[float]]] = []
        if self._closed:
            return out
        for bid, label in zip(self.body_ids, self.labels):
            try:
                lo, hi = p.getAABB(bid, -1, physicsClientId=self.cid)
                lo = np.asarray(lo, dtype=np.float64)
                hi = np.asarray(hi, dtype=np.float64)
                center = ((lo + hi) * 0.5).tolist()
                aabb = (hi - lo).tolist()
                out.append((label, center, aabb))
            except Exception as exc:  # pragma: no cover
                print(f"[physics] bodies_gt rigid '{label}' failed: {exc}", file=sys.stderr)
        for cloth_id, label in zip(self.cloth_ids, self.cloth_labels):
            try:
                v = self._cloth_vertices(cloth_id)
                if v.shape[0] == 0:
                    continue
                lo = v.min(axis=0)
                hi = v.max(axis=0)
                center = ((lo + hi) * 0.5).tolist()
                aabb = (hi - lo).tolist()
                out.append((label, center, aabb))
            except Exception as exc:  # pragma: no cover
                print(f"[physics] bodies_gt cloth '{label}' failed: {exc}", file=sys.stderr)
        return out

    def centers(self) -> np.ndarray:
        """Rigid body centers [M,3] (for airflow wakes); empty -> (0,3)."""
        if self._closed or not self.body_ids:
            return np.zeros((0, 3), dtype=np.float64)
        try:
            cs = [
                p.getBasePositionAndOrientation(b, physicsClientId=self.cid)[0]
                for b in self.body_ids
            ]
            return np.nan_to_num(np.asarray(cs, dtype=np.float64).reshape(-1, 3))
        except Exception:
            return np.zeros((0, 3), dtype=np.float64)

    # ------------------------------------------------------------------- close

    def close(self) -> None:
        """Disconnect the physics client. Idempotent and safe."""
        if self._closed:
            return
        self._closed = True
        try:
            p.disconnect(physicsClientId=self.cid)
        except Exception:
            pass
