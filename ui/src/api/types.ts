// Shapes returned by the Python control-station service. These mirror the JSON
// emitted by drone_control/service.py. Fields are intentionally permissive
// (optional / index signatures) because the service evolves independently.

export interface Connection {
  ssid?: string;
  iface?: string;
  ip?: string;
  control?: string;
  camera?: string;
}

export interface RecordEntry {
  id: string;
  label: string;
  type: string;
  mime?: string;
  path?: string;
  blobKey?: string;
  streamUrl?: string;
}

export interface FlightMetrics {
  frames?: number;
  packets?: number;
  bytes?: number;
  resolution?: string;
  temporalMae?: number;
  smoothedTemporalMae?: number;
  [key: string]: unknown;
}

export interface Flight {
  id: string;
  name: string;
  mode?: string;
  duration?: string;
  startedAt?: string;
  policy?: Record<string, unknown> | string;
  metadata?: Record<string, unknown>;
  metrics?: FlightMetrics;
  records?: RecordEntry[];
}

export interface Drone {
  id: string;
  name: string;
  model: string;
  status: string;
  lastSeen: string;
  connection: Connection;
  flights: Flight[];
}

export interface StationState {
  drones: Drone[];
}

export interface NetworkInterface {
  name: string;
  connection?: string;
}

export interface NetworkSummary {
  platform?: string;
  defaultInterface?: string;
  interfaces?: NetworkInterface[];
  singleWifiLikely?: boolean;
  notes?: string;
}

export interface ManualTransport {
  enabled?: boolean;
  connected?: boolean;
  target?: string;
  lastError?: string;
  linkType?: string;
  iface?: string;
  ip?: string;
  port?: number;
  protocol?: string;
  bindDevice?: boolean;
  ssid?: string;
  password?: string;
  serialPort?: string;
  serialBaud?: number;
}

export interface ManualStatus {
  state: string;
  armed?: boolean;
  faultReason?: string;
  stopReason?: string;
  transport?: ManualTransport;
}

export interface ConfigStatus {
  platform?: string;
  network?: NetworkSummary;
  manual?: ManualTransport;
  policy?: {
    maxThrottle?: number;
    commandHz?: number;
    throttleSlewPerSecond?: number;
    heartbeatTimeoutSeconds?: number;
  };
  reconstruction?: ReconstructionTools;
  runtime?: {
    dryRun?: boolean;
    enableIo?: boolean;
    controlHz?: number;
    localVlaConfigured?: boolean;
    internetVlmConfigured?: boolean;
  };
  linkCapabilities?: Record<string, unknown>;
  camera?: Record<string, unknown>;
}

export interface ReconstructionTools {
  ready?: boolean;
  [key: string]: unknown;
}

export interface SessionStatus {
  running?: boolean;
  frames?: number;
  flightId?: string;
}

export interface ReconstructionJob {
  active?: boolean;
  state?: string;
  stage?: string;
  maxImages?: number;
  maxIterations?: number;
  datasetRecordId?: string;
  splatRecordId?: string;
  error?: string;
  logTail?: string;
}

export interface ReconstructionStatus {
  job?: ReconstructionJob | null;
  latestSplatRecord?: RecordEntry | null;
  tools?: ReconstructionTools;
}

export interface RuntimeSafety {
  armed?: boolean;
  faultReason?: string;
}

export interface RuntimeConstraints {
  maxThrottle?: number;
}

export interface RuntimeObservation {
  confidence?: number;
}

export interface RuntimeDrone {
  droneId: string;
  running?: boolean;
  controller?: string;
  linkType?: string;
  linkState?: string;
  sent?: number;
  errors?: number;
  dryRun?: boolean;
  observation?: RuntimeObservation;
  safety?: RuntimeSafety;
  constraints?: RuntimeConstraints;
  lastAction?: unknown;
}

export interface BatchedVla {
  active?: boolean;
  command?: string;
  batches?: number;
  lastBatchSize?: number;
  maxWaitSeconds?: number;
}

export interface MissionAssignment {
  droneId: string;
  role: string;
  task: string;
}

export interface MissionProgress {
  state?: string;
  assignments?: MissionAssignment[];
  notes?: string[];
}

export interface RuntimeStatus {
  running?: boolean;
  dryRun?: boolean;
  enableIo?: boolean;
  localVlaConfigured?: boolean;
  batchedVlaConfigured?: boolean;
  batchedVla?: BatchedVla;
  drones?: RuntimeDrone[];
  events?: unknown[];
  mission?: MissionProgress;
}

export interface WorldDroneStatus {
  droneId?: string;
  keyframes?: number;
  gaussians?: number;
  [key: string]: unknown;
}

export interface WorldSplatStatus {
  available?: boolean;
  running?: boolean;
  gaussians?: number;
  keyframes?: number;
  keyframesByDrone?: Record<string, number>;
  drones?: WorldDroneStatus[];
  steps?: number;
  lastLoss?: number;
  reason?: string;
}

export interface PoseStatus {
  state?: string;
  fps?: number;
  keyframes?: number;
  scaleLocked?: boolean;
  intrinsicsSource?: string;
  framesAvailable?: boolean;
  estimatorAvailable?: boolean;
}

export interface Pose {
  frameIndex?: number;
  x: number;
  y: number;
  z: number;
  qw: number;
  qx: number;
  qy: number;
  qz: number;
}

export interface PoseTrackResult {
  status?: PoseStatus;
  poses?: Pose[];
}

export interface DiscoverResult {
  state?: StationState;
  discovered?: Array<{ ssid: string }>;
}

export interface SimDrone {
  droneId: string;
  color: string;
  position: number[];
  goal: number[];
  distance: number;
  hasFrame: boolean;
}

export interface SimStatus {
  running?: boolean;
  task?: string;
  numDrones?: number;
  rateHz?: number;
  step?: number;
  render?: boolean;
  drones?: SimDrone[];
}

export interface TrajectoryDrone {
  droneId: string;
  color: string;
  goal: number[] | null;
  poses: Pose[];
}

export interface TrajectoriesResult {
  running?: boolean;
  drones: TrajectoryDrone[];
}
