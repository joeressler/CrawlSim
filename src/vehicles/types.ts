export type Vec3 = {
  x: number;
  y: number;
  z: number;
};

export type WheelId = "fl" | "fr" | "rl" | "rr";

/** Local hardpoint on a kit part (chassis or axle). */
export type MountDef = {
  id: string;
  offset: Vec3;
};

/** Reference to a mount on the chassis or on a named axle. */
export type MountRef = {
  /** `"chassis"` or an axle `id`. */
  part: string;
  mount: string;
};

/**
 * Legacy per-wheel hardpoint for the chassis-ray drive path.
 * `offset` is chassis-local; filled from `axle.offset + hubOffset` when loading a kit.
 */
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

/** Placeholder ids for later swappable chassis / tires / motor / battery / axles. */
export type PartIds = {
  chassis: string;
  tires: string;
  motor: string;
  battery: string;
  axles?: string;
  links?: string;
  shocks?: string;
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

/** Chassis body + mount BOM for links/shocks (Strategy B kit). */
export type ChassisKitDef = {
  halfExtents: Vec3;
  mass: number;
  mounts: MountDef[];
};

/**
 * Solid axle rest pose in chassis frame.
 * Phase 1: data only (no axle rigid body). Later phases attach links/shocks here.
 */
export type AxleDef = {
  id: string;
  /** Axle center in chassis frame at rest. */
  offset: Vec3;
  mounts: MountDef[];
};

/** Control-arm / link / panhard between two mounts. */
export type LinkDef = {
  id: string;
  from: MountRef;
  to: MountRef;
  /** Defaults to arm. Panhard is a lateral locator (still two sphericals — Strategy B). */
  kind?: "arm" | "panhard";
};

/** Coilover between two mounts. Phase 1: rates feed legacy `suspension`; no force solver yet. */
export type ShockDef = {
  id: string;
  from: MountRef;
  to: MountRef;
  restLength: number;
  springK: number;
  damperC: number;
  maxTravel: number;
};

/** Wheel as a hub on an axle. `hubOffset` is axle-local. */
export type KitWheelDef = {
  id: WheelId;
  axle: string;
  hubOffset: Vec3;
  radius: number;
  width: number;
  driven: boolean;
  steered: boolean;
  restLength?: number;
  springK?: number;
  damperC?: number;
  mu?: number;
};

/** Drive / grip tunables shared by the chassis-ray path (and later axle drive). */
export type DriveDef = {
  mu: number;
  driveTorque: number;
  maxForce: number;
  steerAngle: number;
  maxAccel: number;
  minNormalY: number;
  minUpright: number;
};

/** Strategy B kit bill of materials. Axle bodies / joints / shock forces come in later phases. */
export type KitDef = {
  chassis: ChassisKitDef;
  axles: AxleDef[];
  links: LinkDef[];
  shocks: ShockDef[];
  wheels: KitWheelDef[];
  drive: DriveDef;
};

/**
 * Runtime rig consumed by CrawlerVehicle / drive.ts.
 * Always has legacy `chassis` / `wheels` / `suspension` after `loadRig`.
 * When JSON carries `kit`, the loader validates mounts and fills those legacy fields.
 */
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
  /** Present when the source JSON used the Strategy B kit BOM. */
  kit?: KitDef;
};

/** On-disk shape: either kit BOM, legacy chassis-ray fields, or both. */
export type RawRigJson = {
  parts: PartIds;
  spawn: Vec3;
  maxSpeed: number;
  cameraOffset: Vec3;
  kit?: KitDef;
  chassis?: {
    halfExtents: Vec3;
    mass: number;
  };
  wheels?: WheelDef[];
  suspension?: SuspensionDef;
};