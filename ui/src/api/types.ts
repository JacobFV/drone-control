// Shapes returned by the session-centric Python service (drone_control/service.py
// + session_service.py). Permissive on purpose — the backend evolves.

export interface Pose {
  frameIndex?: number;
  x: number;
  y: number;
  z: number;
  qw?: number;
  qx?: number;
  qy?: number;
  qz?: number;
}

export interface TrajectoryDrone {
  droneId: string;
  color: string;
  goal: number[] | null;
  poses: Pose[];
}

/** One drone's VO-estimated trajectory, similarity-aligned to ground truth. */
export interface EstimatedTrajectoryDrone {
  droneId: string;
  color?: string | null;
  poses: Pose[];
  aligned: boolean; // true when aligned to ground truth (sim); raw otherwise
  scale: number | null; // similarity scale applied during alignment
  driftRmse: number | null; // absolute-pose-error RMSE after alignment (m)
  driftFinal: number | null; // current-position error after alignment (m)
  state: string; // VO tracking state (tracking | degraded | awaiting_parallax | …)
  confidence: number;
  keyframes: number;
}

export interface EstimatedTrajectories {
  available: boolean;
  reason?: string | null;
  drones: EstimatedTrajectoryDrone[];
}

export interface RecordEntry {
  id: string;
  sessionId?: string;
  droneId?: string | null;
  source: string; // camera | pose | control | splat | seg-screen | seg-world | artifact
  type: string;
  label: string;
  mime?: string;
  byteCount?: number;
  path?: string;
  blobKey?: string;
  streamUrl?: string;
  poseUrl?: string;
  metadata?: Record<string, unknown>;
}

export interface Session {
  id: string;
  environmentId: string;
  name: string;
  state: string;
  drones: string[];
  startedAt?: string;
  endedAt?: string | null;
  duration?: string;
  metadata?: Record<string, unknown>;
  metrics?: Record<string, unknown>;
  records: RecordEntry[];
}

export interface Environment {
  id: string;
  name: string;
  kind: string; // sim | real
  config?: Record<string, unknown>;
  createdAt?: string;
  sessions: Session[];
}

export interface Drone {
  id: string;
  name: string;
  model: string;
  status: string;
  lastSeen: string;
  identity?: Record<string, unknown>;
  connection?: Record<string, unknown>;
}

export interface StationState {
  environments: Environment[];
  drones: Drone[];
}

export interface ScreenDetection {
  cls: string;
  score: number;
  bbox: number[]; // [x,y,w,h] px
  centroid: number[]; // [cx,cy] px
  polygon: number[][]; // normalized [[x,y],...]
  width: number;
  height: number;
}

export interface WorldObject {
  id: number;
  cls: string;
  centroid: number[]; // [x,y,z]
  count: number;
  drones: string[];
  score: number;
}

export interface SegmentationStatus {
  available: boolean;
  reason?: string | null;
  model?: string;
  objects?: number;
  dronesWithScreen?: string[];
}

/** Stick command (E99 byte form, neutral 128) sent to a drone. */
export interface DroneCommand {
  roll: number;
  pitch: number;
  throttle: number;
  yaw: number;
}

/** One history entry in a drone's command log (a sampled command or an event). */
export interface DroneCommandEntry {
  t: number;
  roll?: number;
  pitch?: number;
  throttle?: number;
  yaw?: number;
  event?: string;
}

/** Per-drone live status inside a sim session's `env.drones`. */
export interface SimDrone {
  droneId: string;
  color?: string;
  position?: number[];
  goal?: number[];
  distance?: number;
  hasFrame?: boolean;
  command?: DroneCommand | null;
  frozen?: boolean;
}

/** Command history + record directory for one drone (Drones panel). */
export interface DroneDetail {
  droneId: string;
  commands: DroneCommandEntry[];
  dir?: string | null;
  frameCount?: number;
}

export interface SessionStatus {
  active: boolean;
  sessionId?: string;
  environmentId?: string;
  kind?: string;
  recording?: boolean;
  speed?: "realtime" | "max";
  paused?: boolean;
  elapsedSeconds?: number;
  drones?: string[];
  frameCounts?: Record<string, number>;
  trajectories?: TrajectoryDrone[];
  estimatedTrajectories?: EstimatedTrajectories;
  worldModel?: { available?: boolean; running?: boolean; gaussians?: number; reason?: string };
  segmentation?: {
    status: SegmentationStatus;
    screen: Record<string, ScreenDetection[]>;
    world: WorldObject[];
  };
  depth?: {
    available: boolean;
    reason?: string | null;
    model?: string;
    points?: number;
    dronesWithDepth?: string[];
  };
  env?: Record<string, unknown>;
  metrics?: Record<string, unknown>;
}

export interface RuntimeDrone {
  droneId: string;
  running?: boolean;
  controller?: string;
  linkState?: string;
  sent?: number;
  errors?: number;
  lastAction?: DroneCommand & { takeoff?: boolean; land?: boolean; emergency_stop?: boolean };
  safety?: { armed?: boolean; faultReason?: string };
}

export interface RuntimeStatus {
  running?: boolean;
  drones?: RuntimeDrone[];
  mission?: { state?: string; notes?: string[] };
}

export interface ManualStatus {
  state: string;
  armed?: boolean;
  faultReason?: string;
  transport?: Record<string, unknown>;
}

export interface WsSnapshot {
  session: SessionStatus;
  runtime: RuntimeStatus;
  manual: ManualStatus;
}

export interface NetworkSummary {
  platform?: string;
  defaultInterface?: string;
  interfaces?: { name: string; connection?: string }[];
  notes?: string;
}

/** A usable PC Wi-Fi radio (one association per drone AP). */
export interface WifiInterface {
  name: string;
  state: string; // connected | disconnected | …
  connection: string; // currently-joined SSID, "" when idle
  platform?: string;
  kind?: string; // wifi
}

/** A USB serial device that can act as an ESP32 drone-link bridge. */
export interface SerialBridge {
  port: string; // /dev/ttyACM0
  by_id: string; // stable /dev/serial/by-id path, "" if unknown
  serial: string;
  vendor_id: string;
  product_id: string;
  manufacturer: string;
  product: string;
  is_esp: boolean;
}

/** A network seen by a radio's scan. */
export interface AccessPoint {
  ssid: string;
  bssid: string;
  channel: string;
  frequency: string;
  signal: number; // 0–100
  security: string;
  likely_drone: boolean;
}

export interface ConfigStatus {
  platform?: string;
  network?: NetworkSummary;
  manual?: Record<string, unknown>;
  policy?: Record<string, unknown>;
  camera?: Record<string, unknown>;
  reconstruction?: { ready?: boolean; [k: string]: unknown };
  runtime?: Record<string, unknown>;
}

export interface DiscoverResult {
  discovered?: { ssid: string }[];
  state?: StationState;
}
