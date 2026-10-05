import stock from "./rigs/stock.json" with { type: "json" };
import type {
  AxleDef,
  KitDef,
  MountDef,
  TireDef,
  MountRef,
  RawRigJson,
  RigDef,
  SuspensionDef,
  Vec3,
  WheelDef,
} from "../vehicles/types.ts";

function validateTire(tire: TireDef): void {
  if (!(tire.radialK > 0)) throw new Error("kit.tire.radialK must be > 0");
  if (!(tire.radialC >= 0)) throw new Error("kit.tire.radialC must be >= 0");
  if (!(tire.reboundC >= 0)) throw new Error("kit.tire.reboundC must be >= 0");
  if (!(tire.maxDeflection > 0.05 && tire.maxDeflection < 0.65)) {
    throw new Error("kit.tire.maxDeflection must be between 0.05 and 0.65");
  }
  if (!(tire.deflectionFilter > 0 && tire.deflectionFilter < 0.1)) {
    throw new Error("kit.tire.deflectionFilter must be between 0 and 0.1 s");
  }
}

function addVec(a: Vec3, b: Vec3): Vec3 {
  return { x: a.x + b.x, y: a.y + b.y, z: a.z + b.z };
}

function mountIndex(mounts: MountDef[]): Map<string, MountDef> {
  const map = new Map<string, MountDef>();
  for (const mount of mounts) {
    if (map.has(mount.id)) {
      throw new Error(`duplicate mount id "${mount.id}"`);
    }
    map.set(mount.id, mount);
  }
  return map;
}

function resolveMount(
  ref: MountRef,
  chassisMounts: Map<string, MountDef>,
  axles: Map<string, AxleDef>,
  context: string
): void {
  if (ref.part === "chassis") {
    if (!chassisMounts.has(ref.mount)) {
      throw new Error(`${context}: unknown chassis mount "${ref.mount}"`);
    }
    return;
  }
  const axle = axles.get(ref.part);
  if (!axle) {
    throw new Error(`${context}: unknown part "${ref.part}" (expected chassis or axle id)`);
  }
  if (!axle.mounts.some((m) => m.id === ref.mount)) {
    throw new Error(`${context}: unknown mount "${ref.mount}" on axle "${ref.part}"`);
  }
}

function validateKit(kit: KitDef): Map<string, AxleDef> {
  const chassisMounts = mountIndex(kit.chassis.mounts);
  const axles = new Map<string, AxleDef>();
  for (const axle of kit.axles) {
    if (axles.has(axle.id)) {
      throw new Error(`duplicate axle id "${axle.id}"`);
    }
    mountIndex(axle.mounts);
    axles.set(axle.id, axle);
  }

  for (const link of kit.links) {
    resolveMount(link.from, chassisMounts, axles, `link "${link.id}" from`);
    resolveMount(link.to, chassisMounts, axles, `link "${link.id}" to`);
  }
  for (const shock of kit.shocks) {
    resolveMount(shock.from, chassisMounts, axles, `shock "${shock.id}" from`);
    resolveMount(shock.to, chassisMounts, axles, `shock "${shock.id}" to`);
  }
  if (kit.shocks.length === 0) {
    throw new Error("kit.shocks must include at least one shock to seed legacy suspension");
  }
  validateTire(kit.tire);

  const wheelIds = new Set<string>();
  for (const wheel of kit.wheels) {
    if (wheelIds.has(wheel.id)) {
      throw new Error(`duplicate kit wheel id "${wheel.id}"`);
    }
    wheelIds.add(wheel.id);
    if (!axles.has(wheel.axle)) {
      throw new Error(`wheel "${wheel.id}" references unknown axle "${wheel.axle}"`);
    }
  }
  return axles;
}

/** Average shock rates → shared legacy suspension; drive supplies grip / motor caps. */
function suspensionFromKit(kit: KitDef): SuspensionDef {
  const n = kit.shocks.length;
  let restLength = 0;
  let springK = 0;
  let damperC = 0;
  let maxTravel = 0;
  for (const shock of kit.shocks) {
    restLength += shock.restLength;
    springK += shock.springK;
    damperC += shock.damperC;
    maxTravel += shock.maxTravel;
  }
  const { drive } = kit;
  return {
    restLength: restLength / n,
    springK: springK / n,
    damperC: damperC / n,
    maxTravel: maxTravel / n,
    mu: drive.mu,
    driveTorque: drive.driveTorque,
    maxForce: drive.maxForce,
    steerAngle: drive.steerAngle,
    maxAccel: drive.maxAccel,
    minNormalY: drive.minNormalY,
    minUpright: drive.minUpright,
  };
}

function wheelsFromKit(kit: KitDef, axles: Map<string, AxleDef>): WheelDef[] {
  return kit.wheels.map((wheel) => {
    const axle = axles.get(wheel.axle);
    if (!axle) {
      throw new Error(`wheel "${wheel.id}" references unknown axle "${wheel.axle}"`);
    }
    const def: WheelDef = {
      id: wheel.id,
      offset: addVec(axle.offset, wheel.hubOffset),
      radius: wheel.radius,
      width: wheel.width,
      driven: wheel.driven,
      steered: wheel.steered,
    };
    if (wheel.restLength !== undefined) def.restLength = wheel.restLength;
    if (wheel.springK !== undefined) def.springK = wheel.springK;
    if (wheel.damperC !== undefined) def.damperC = wheel.damperC;
    if (wheel.mu !== undefined) def.mu = wheel.mu;
    return def;
  });
}

/**
 * Dual-read loader:
 * - Kit BOM present → validate mounts, fill legacy chassis / wheels.offset / suspension.
 * - Legacy-only JSON → pass through unchanged (no kit on RigDef).
 * A kit rig drives through the axle patch. Legacy JSON still uses the chassis-ray path.
 */
export function loadRig(raw: RawRigJson): RigDef {
  const parts = {
    ...raw.parts,
    servo: raw.parts.servo ?? raw.parts.battery ?? "stock-servo",
  };

  if (raw.kit) {
    const axles = validateKit(raw.kit);
    return {
      parts,
      initialSceneId: raw.initialSceneId,
      kit: raw.kit,
      chassis: {
        halfExtents: raw.kit.chassis.halfExtents,
        mass: raw.kit.chassis.mass,
      },
      spawn: raw.spawn,
      wheels: wheelsFromKit(raw.kit, axles),
      suspension: suspensionFromKit(raw.kit),
      maxSpeed: raw.maxSpeed,
      cameraOffset: raw.cameraOffset,
    };
  }

  if (!raw.chassis || !raw.wheels || !raw.suspension) {
    throw new Error("rig JSON needs either kit or legacy chassis + wheels + suspension");
  }
  return {
    parts,
    initialSceneId: raw.initialSceneId,
    chassis: raw.chassis,
    spawn: raw.spawn,
    wheels: raw.wheels,
    suspension: raw.suspension,
    maxSpeed: raw.maxSpeed,
    cameraOffset: raw.cameraOffset,
  };
}

export function loadStockRig(): RigDef {
  return loadRig(stock as RawRigJson);
}