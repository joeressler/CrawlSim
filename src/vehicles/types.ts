export type Vec3 = {
  x: number;
  y: number;
  z: number;
};

export type WheelId = "fl" | "fr" | "rl" | "rr";

export type WheelDef = {
  id: WheelId;
  offset: Vec3;
  radius: number;
  width: number;
  driven: boolean;
  steered: boolean;
  /** Overrides `suspension` for this tire when set. */
  restLength?: number;
  springK?: number;
  damperC?: number;
  mu?: number;
};

/** Placeholder ids for later swappable chassis / tires / motor / battery. */
export type PartIds = {
  chassis: string;
  tires: string;
  motor: string;
  battery: string;
};

/** Shared crawl suspension and tire grip. Per-wheel fields override these. */
export type SuspensionDef = {
  restLength: number;
  springK: number;
  damperC: number;
  mu: number;
  /** Motor torque stub (N·m). Longitudinal force is capped by `driveTorque / radius`. */
  driveTorque: number;
  /** Absolute cap on the friction vector magnitude (N), alongside μ·Fn. */
  maxForce: number;
  /** Front-wheel steer limit, radians. */
  steerAngle: number;
  /** How fast commanded speed can change (m/s²). Keeps throttle and reverse from pitching the chassis. */
  maxAccel: number;
  /** Ignore contacts flatter than this world-up component so ledge faces do not grab. */
  minNormalY: number;
  /** How far the strut may compress from `restLength` (m). Extension never exceeds restLength. */
  maxTravel: number;
  /** Chassis-up · world-up below this disables tire support/drive (no upside-down crawling). */
  minUpright: number;
};

export type RigDef = {
  parts: PartIds;
  chassis: {
    halfExtents: Vec3;
    mass: number;
  };
  spawn: Vec3;
  wheels: WheelDef[];
  suspension: SuspensionDef;
  maxSpeed: number;
  cameraOffset: Vec3;
};
