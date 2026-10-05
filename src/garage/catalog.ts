import type { KitDef, RigDef } from "../vehicles/types.ts";

export type ShockProfileId = "stock" | "soft-trail" | "firm-race";
export type WheelProfileId = "stock" | "trail-grip" | "hardpack";
export type MotorProfileId = "stock" | "crawl-torque" | "sport-speed";
export type ServoProfileId = "stock" | "precision" | "quick";

export type GarageConfig = {
  shockProfileId: ShockProfileId;
  wheelProfileId: WheelProfileId;
  motorProfileId: MotorProfileId;
  servoProfileId: ServoProfileId;
  linkColor: string;
  shockColor: string;
  servoColor: string;
};

export const DEFAULT_GARAGE_CONFIG: GarageConfig = {
  shockProfileId: "stock",
  wheelProfileId: "stock",
  motorProfileId: "stock",
  servoProfileId: "stock",
  linkColor: "#b08d57",
  shockColor: "#1e4d7a",
  servoColor: "#d14f3f",
};

const STORAGE_KEY = "crawlsim.garage.config.v1";

type ProfileOption<T extends string> = {
  id: T;
  label: string;
  summary: string;
};

export const SHOCK_OPTIONS: ProfileOption<ShockProfileId>[] = [
  { id: "stock", label: "Stock Coilovers", summary: "Balanced trail control, stock travel." },
  { id: "soft-trail", label: "Soft Trail", summary: "More wheel follow, smoother landings, softer body control." },
  { id: "firm-race", label: "Firm Race", summary: "Tighter body control, less squat, steeper response." },
];

export const WHEEL_OPTIONS: ProfileOption<WheelProfileId>[] = [
  { id: "stock", label: "Stock 1.9 Trail", summary: "Balanced grip and compliance for all-round trails." },
  { id: "trail-grip", label: "High Grip Trail", summary: "More bite on rocks and ledges, smoother climb traction." },
  { id: "hardpack", label: "Hardpack / Rock", summary: "Sharper rebound, less flex, more direct steering response." },
];

export const MOTOR_OPTIONS: ProfileOption<MotorProfileId>[] = [
  { id: "stock", label: "Stock Crawl", summary: "Measured torque and speed for controlled crawling." },
  { id: "crawl-torque", label: "High Torque Crawl", summary: "Improved low-speed pull; less top-end urgency." },
  { id: "sport-speed", label: "Sport Speed", summary: "Higher top speed and acceleration, less crawl finesse." },
];

export const SERVO_OPTIONS: ProfileOption<ServoProfileId>[] = [
  { id: "stock", label: "Stock Servo", summary: "Steady steering feel for stock handling." },
  { id: "precision", label: "Precision Servo", summary: "Slower but more deliberate steering corrections." },
  { id: "quick", label: "Quick Servo", summary: "Faster steering changes, slightly less controlled bite." },
];

type ShockProfile = {
  springK: number;
  damperC: number;
  restLength: number;
  maxTravel: number;
};

type WheelProfile = {
  mu: number;
  radialK: number;
  radialC: number;
  reboundC: number;
  maxDeflection: number;
  radius: number;
  width: number;
};

type MotorProfile = {
  driveTorque: number;
  maxAccel: number;
  maxForce: number;
  maxSpeed: number;
};

type ServoProfile = {
  steerRate: number;
  steerAngle: number;
};

const SHOCK_PROFILES: Record<ShockProfileId, ShockProfile> = {
  stock: { springK: 1, damperC: 1, restLength: 1, maxTravel: 1 },
  "soft-trail": { springK: 0.78, damperC: 0.84, restLength: 1.05, maxTravel: 1.1 },
  "firm-race": { springK: 1.32, damperC: 1.28, restLength: 0.97, maxTravel: 0.88 },
};

const WHEEL_PROFILES: Record<WheelProfileId, WheelProfile> = {
  stock: { mu: 1, radialK: 1, radialC: 1, reboundC: 1, maxDeflection: 1, radius: 1, width: 1 },
  "trail-grip": { mu: 1.2, radialK: 0.96, radialC: 0.95, reboundC: 0.92, maxDeflection: 1.12, radius: 1.02, width: 1.06 },
  hardpack: { mu: 0.9, radialK: 1.18, radialC: 1.12, reboundC: 1.08, maxDeflection: 0.86, radius: 0.98, width: 0.95 },
};

const MOTOR_PROFILES: Record<MotorProfileId, MotorProfile> = {
  stock: { driveTorque: 1, maxAccel: 1, maxForce: 1, maxSpeed: 1 },
  "crawl-torque": { driveTorque: 1.28, maxAccel: 0.92, maxForce: 1.22, maxSpeed: 0.9 },
  "sport-speed": { driveTorque: 1.06, maxAccel: 1.36, maxForce: 1.12, maxSpeed: 1.32 },
};

const SERVO_PROFILES: Record<ServoProfileId, ServoProfile> = {
  stock: { steerRate: 12, steerAngle: 1 },
  precision: { steerRate: 8, steerAngle: 1.08 },
  quick: { steerRate: 18, steerAngle: 0.94 },
};

function isHexColor(value: string): boolean {
  return /^#[0-9a-fA-F]{6}$/.test(value);
}

function sanitizeConfig(input: Partial<GarageConfig>): GarageConfig {
  const shockProfileId = SHOCK_PROFILES[input.shockProfileId as ShockProfileId]
    ? (input.shockProfileId as ShockProfileId)
    : DEFAULT_GARAGE_CONFIG.shockProfileId;
  const wheelProfileId = WHEEL_PROFILES[input.wheelProfileId as WheelProfileId]
    ? (input.wheelProfileId as WheelProfileId)
    : DEFAULT_GARAGE_CONFIG.wheelProfileId;
  const motorProfileId = MOTOR_PROFILES[input.motorProfileId as MotorProfileId]
    ? (input.motorProfileId as MotorProfileId)
    : DEFAULT_GARAGE_CONFIG.motorProfileId;
  const servoProfileId = SERVO_PROFILES[input.servoProfileId as ServoProfileId]
    ? (input.servoProfileId as ServoProfileId)
    : DEFAULT_GARAGE_CONFIG.servoProfileId;

  const linkColor = typeof input.linkColor === "string" && isHexColor(input.linkColor)
    ? input.linkColor
    : DEFAULT_GARAGE_CONFIG.linkColor;
  const shockColor = typeof input.shockColor === "string" && isHexColor(input.shockColor)
    ? input.shockColor
    : DEFAULT_GARAGE_CONFIG.shockColor;
  const servoColor = typeof input.servoColor === "string" && isHexColor(input.servoColor)
    ? input.servoColor
    : DEFAULT_GARAGE_CONFIG.servoColor;

  return { shockProfileId, wheelProfileId, motorProfileId, servoProfileId, linkColor, shockColor, servoColor };
}

export function loadGarageConfig(): GarageConfig {
  if (typeof localStorage === "undefined") return { ...DEFAULT_GARAGE_CONFIG };
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (!raw) return { ...DEFAULT_GARAGE_CONFIG };
    const parsed = JSON.parse(raw) as Partial<GarageConfig>;
    return sanitizeConfig(parsed);
  } catch {
    return { ...DEFAULT_GARAGE_CONFIG };
  }
}

export function saveGarageConfig(config: GarageConfig): void {
  if (typeof localStorage === "undefined") return;
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(config));
  } catch {
    // Best-effort persistence only.
  }
}

function applyShockProfile(kit: KitDef, profile: ShockProfile): void {
  for (const shock of kit.shocks) {
    shock.springK *= profile.springK;
    shock.damperC *= profile.damperC;
    shock.restLength *= profile.restLength;
    shock.maxTravel *= profile.maxTravel;
  }
}

function applyWheelProfile(kit: KitDef, profile: WheelProfile): void {
  kit.drive.mu *= profile.mu;
  kit.tire.radialK *= profile.radialK;
  kit.tire.radialC *= profile.radialC;
  kit.tire.reboundC *= profile.reboundC;
  kit.tire.maxDeflection *= profile.maxDeflection;
  for (const wheel of kit.wheels) {
    wheel.radius *= profile.radius;
    wheel.width *= profile.width;
  }
}

function applyMotorProfile(rig: RigDef, kit: KitDef, profile: MotorProfile): void {
  kit.drive.driveTorque *= profile.driveTorque;
  kit.drive.maxAccel *= profile.maxAccel;
  kit.drive.maxForce *= profile.maxForce;
  rig.maxSpeed *= profile.maxSpeed;
}

function applyServoProfile(kit: KitDef, profile: ServoProfile): void {
  kit.drive.steerRate = profile.steerRate;
  kit.drive.steerAngle *= profile.steerAngle;
}

export function getSelectedOptionSummary<T extends string>(
  options: ProfileOption<T>[],
  selected: T,
): string {
  const option = options.find((item) => item.id === selected);
  return option?.summary ?? "Balanced stock tuning.";
}

export function applyGarageConfigToRig(baseRig: RigDef, config: GarageConfig): RigDef {
  const rig = structuredClone(baseRig);
  rig.parts.shocks = config.shockProfileId;
  rig.parts.tires = config.wheelProfileId;
  rig.parts.motor = config.motorProfileId;
  rig.parts.servo = config.servoProfileId;

  const kit = rig.kit;
  if (!kit) return rig;

  applyShockProfile(kit, SHOCK_PROFILES[config.shockProfileId]);
  applyWheelProfile(kit, WHEEL_PROFILES[config.wheelProfileId]);
  applyMotorProfile(rig, kit, MOTOR_PROFILES[config.motorProfileId]);
  applyServoProfile(kit, SERVO_PROFILES[config.servoProfileId]);

  return rig;
}
