// 3D pose/trajectory math, ported verbatim from the original app/renderer.js
// (lines ~1245-1396). Right-handed world, Z-up. The estimator track is rendered
// onto a 2D canvas with a simple pinhole projection and an orbit camera.

export type Vec3 = [number, number, number];

export interface OrbitView {
  yaw: number;
  pitch: number;
  distance: number;
  target: Vec3;
}

export interface ViewMatrix {
  eye: Vec3;
  target: Vec3;
  up: Vec3;
}

export const DEFAULT_VIEW: OrbitView = {
  yaw: -Math.PI / 6,
  pitch: Math.PI / 5,
  distance: 60,
  target: [0, 0, 0],
};

export function sub(a: Vec3, b: Vec3): Vec3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}
export function add(a: Vec3, b: Vec3): Vec3 {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}
export function dot(a: Vec3, b: Vec3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
export function cross(a: Vec3, b: Vec3): Vec3 {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}
export function normalize(v: Vec3): Vec3 {
  const len = Math.hypot(v[0], v[1], v[2]) || 1;
  return [v[0] / len, v[1] / len, v[2] / len];
}

export function computeViewMatrix(view: OrbitView): ViewMatrix {
  const cy = Math.cos(view.yaw);
  const sy = Math.sin(view.yaw);
  const cp = Math.cos(view.pitch);
  const sp = Math.sin(view.pitch);
  const eyeX = view.target[0] + view.distance * cp * sy;
  const eyeY = view.target[1] + view.distance * cp * cy;
  const eyeZ = view.target[2] + view.distance * sp;
  return { eye: [eyeX, eyeY, eyeZ], target: [...view.target], up: [0, 0, 1] };
}

export function projectPoint(
  p: Vec3,
  view: ViewMatrix,
  focal: number,
  w: number,
  h: number,
): [number, number, number] | null {
  const fwd = normalize(sub(view.target, view.eye));
  const right = normalize(cross(fwd, view.up));
  const up = cross(right, fwd);
  const rel = sub(p, view.eye);
  const xc = dot(rel, right);
  const yc = -dot(rel, up);
  const zc = dot(rel, fwd);
  if (zc <= 0.001) return null;
  return [w / 2 + (focal * xc) / zc, h / 2 + (focal * yc) / zc, zc];
}

export function quatToMatrix(
  w: number,
  x: number,
  y: number,
  z: number,
): number[][] {
  const n = Math.hypot(w, x, y, z) || 1;
  w /= n;
  x /= n;
  y /= n;
  z /= n;
  return [
    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
  ];
}

export function applyR(R: number[][], v: Vec3): Vec3 {
  return [
    R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
    R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
    R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
  ];
}

/** Pan the orbit target in screen space (drag with shift / middle / right). */
export function panView(view: OrbitView, dx: number, dy: number, viewportMin: number): void {
  const vm = computeViewMatrix(view);
  const fwd = normalize(sub(vm.target, vm.eye));
  const right = normalize(cross(fwd, vm.up));
  const up = cross(right, fwd);
  const k = view.distance / Math.max(1, viewportMin);
  for (let i = 0; i < 3; i += 1) {
    view.target[i] -= dx * k * right[i] - dy * k * up[i];
  }
}
