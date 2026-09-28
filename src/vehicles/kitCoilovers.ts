/**
 * Phase 3 coilovers between chassis/axle shock mounts.
 * restLength = geometric mount distance at build (must match BOM so no self-push).
 * Forces along chassis-up only. Compress-only spring (no extension pull fighting links).
 * Hub spheres = only plant; no Rapier spring joints.
 */
import type RAPIER from "@dimforge/rapier3d-compat";
import * as THREE from "three";
import type { KitDef, ShockDef, Vec3 } from "./types.ts";
import type { KitAxleRuntime } from "./kitBodies.ts";

export type CoiloverRuntime = {
  def: ShockDef;
  restLength: number;
  maxTravel: number;
  springK: number;
  damperC: number;
  forceCap: number;
  chassisMount: Vec3;
  axleId: string;
  axleMount: Vec3;
};

const scratchQ = new THREE.Quaternion();
const scratchV = new THREE.Vector3();
const up = new THREE.Vector3();

function worldPoint(body: RAPIER.RigidBody, local: Vec3): { x: number; y: number; z: number } {
  const t = body.translation();
  const r = body.rotation();
  scratchQ.set(r.x, r.y, r.z, r.w);
  scratchV.set(local.x, local.y, local.z).applyQuaternion(scratchQ);
  return { x: t.x + scratchV.x, y: t.y + scratchV.y, z: t.z + scratchV.z };
}

function mountOn(kit: KitDef, part: string, mountId: string): Vec3 {
  if (part === "chassis") {
    const m = kit.chassis.mounts.find((x) => x.id === mountId);
    if (!m) throw new Error(`coilover: missing chassis mount ${mountId}`);
    return m.offset;
  }
  const axle = kit.axles.find((a) => a.id === part);
  if (!axle) throw new Error(`coilover: missing axle ${part}`);
  const m = axle.mounts.find((x) => x.id === mountId);
  if (!m) throw new Error(`coilover: missing mount ${mountId} on ${part}`);
  return m.offset;
}

export function buildCoilovers(
  kit: KitDef,
  chassis: RAPIER.RigidBody,
  axles: Map<string, KitAxleRuntime>
): CoiloverRuntime[] {
  const out: CoiloverRuntime[] = [];
  const chassisMass = chassis.mass();
  for (const shock of kit.shocks) {
    if (shock.from.part !== "chassis") {
      throw new Error(`coilover ${shock.id}: from must be chassis`);
    }
    const axleId = shock.to.part;
    const axle = axles.get(axleId);
    if (!axle) throw new Error(`coilover ${shock.id}: unknown axle ${axleId}`);
    const chassisMount = mountOn(kit, shock.from.part, shock.from.mount);
    const axleMount = mountOn(kit, shock.to.part, shock.to.mount);
    const a = worldPoint(chassis, chassisMount);
    const b = worldPoint(axle.body, axleMount);
    // Rest = exact mount separation at build — never fight links with a fake rest.
    const restLength = Math.max(0.04, Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z));
    const springK = shock.springK;
    const mEff = Math.max(0.5, chassisMass / Math.max(1, kit.shocks.length));
    const critical = 2 * Math.sqrt(springK * mEff);
    const damperC = Math.max(shock.damperC, critical * 1.8);
    const maxTravel = Math.min(shock.maxTravel, restLength * 0.4);
    const forceCap = (2.8 * chassisMass * 9.81) / Math.max(1, kit.shocks.length);
    out.push({
      def: shock,
      restLength,
      maxTravel,
      springK,
      damperC,
      forceCap,
      chassisMount,
      axleId,
      axleMount,
    });
  }
  return out;
}

export function applyCoilovers(
  chassis: RAPIER.RigidBody,
  axles: Map<string, KitAxleRuntime>,
  coilovers: CoiloverRuntime[],
  dt: number
): void {
  if (dt <= 0 || coilovers.length === 0) return;

  const rot = chassis.rotation();
  scratchQ.set(rot.x, rot.y, rot.z, rot.w);
  up.set(0, 1, 0).applyQuaternion(scratchQ);
  const ux = up.x;
  const uy = up.y;
  const uz = up.z;

  for (const coil of coilovers) {
    const axle = axles.get(coil.axleId);
    if (!axle) continue;
    const p0 = worldPoint(chassis, coil.chassisMount);
    const p1 = worldPoint(axle.body, coil.axleMount);

    // Use full mount distance vs rest so lateral mount offset does not invent compression.
    const dist = Math.hypot(p0.x - p1.x, p0.y - p1.y, p0.z - p1.z);
    const compression = coil.restLength - dist;

    const v0 = chassis.velocityAtPoint(p0);
    const v1 = axle.body.velocityAtPoint(p1);
    // Closing along chassis-up (compress-positive).
    const closingSpeed = (v1.x - v0.x) * ux + (v1.y - v0.y) * uy + (v1.z - v0.z) * uz;

    let force = 0;
    if (compression > 0) {
      force = coil.springK * compression + coil.damperC * closingSpeed;
      const bumpDepth = compression - coil.maxTravel;
      if (bumpDepth > 0) {
        force += coil.springK * 8 * bumpDepth + coil.damperC * 2 * Math.max(0, closingSpeed);
      }
    } else {
      // Extension: damper only — no spring pull fighting links / panhard.
      force = coil.damperC * 0.35 * closingSpeed;
    }

    if (force > coil.forceCap) force = coil.forceCap;
    if (force < -coil.forceCap * 0.35) force = -coil.forceCap * 0.35;

    const impulse = force * dt;
    chassis.applyImpulseAtPoint({ x: ux * impulse, y: uy * impulse, z: uz * impulse }, p0, true);
    axle.body.applyImpulseAtPoint({ x: -ux * impulse, y: -uy * impulse, z: -uz * impulse }, p1, true);
  }
}
