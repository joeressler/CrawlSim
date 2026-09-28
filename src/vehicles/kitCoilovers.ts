/**
 * Phase 3 coilovers between chassis/axle shock mounts.
 * restLength = geometric mount distance at build + small hang bias.
 * Forces along world-up (chassis-up coupled into roll on soft sphericals).
 * L/R anti-roll bar restores roll stiffness; compress-only spring.
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
    // Small rest bias: world-up plant sits lower; keep chassis hanging above axles.
    const restBias = axleId === "front" ? 0.024 : 0.008;
    const restLength = Math.max(0.04, Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z) + restBias);
    const springK = shock.springK;
    const mEff = Math.max(0.5, chassisMass / Math.max(1, kit.shocks.length));
    const critical = 2 * Math.sqrt(springK * mEff);
    const damperC = Math.max(shock.damperC, critical * 2.4);
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

  // World-up: chassis-up couples into roll when soft sphericals let rails twist
  // on planted hubs. ARB restores roll stiffness without chassis torque.
  const ux = 0;
  const uy = 1;
  const uz = 0;

  type Sample = {
    coil: CoiloverRuntime;
    axle: KitAxleRuntime;
    p0: { x: number; y: number; z: number };
    p1: { x: number; y: number; z: number };
    compression: number;
    closingSpeed: number;
    force: number;
  };
  const samples: Sample[] = [];

  for (const c of coilovers) {
    const axle = axles.get(c.axleId);
    if (!axle) continue;
    const p0 = worldPoint(chassis, c.chassisMount);
    const p1 = worldPoint(axle.body, c.axleMount);
    const dist = Math.hypot(p0.x - p1.x, p0.y - p1.y, p0.z - p1.z);
    const compression = c.restLength - dist;

    const v0 = chassis.velocityAtPoint(p0);
    const v1 = axle.body.velocityAtPoint(p1);
    const closingSpeed = (v1.x - v0.x) * ux + (v1.y - v0.y) * uy + (v1.z - v0.z) * uz;

    let force = 0;
    if (compression > 0) {
      force = c.springK * compression + c.damperC * closingSpeed;
      const bumpDepth = compression - c.maxTravel;
      if (bumpDepth > 0) {
        force += c.springK * 8 * bumpDepth + c.damperC * 2 * Math.max(0, closingSpeed);
      }
    } else {
      // Extension: damper only — no spring pull fighting links / panhard.
      force = c.damperC * 0.35 * closingSpeed;
    }
    if (force > c.forceCap) force = c.forceCap;
    if (force < -c.forceCap * 0.35) force = -c.forceCap * 0.35;
    samples.push({ coil: c, axle, p0, p1, compression, closingSpeed, force });
  }

  const byAxle = new Map<string, Sample[]>();
  for (const s of samples) {
    const list = byAxle.get(s.coil.axleId) ?? [];
    list.push(s);
    byAxle.set(s.coil.axleId, list);
  }
  const ARB_K = 55;
  const ARB_C = 40;
  for (const pair of byAxle.values()) {
    if (pair.length !== 2) continue;
    const left = pair.find((s) => s.coil.chassisMount.x < 0);
    const right = pair.find((s) => s.coil.chassisMount.x > 0);
    if (!left || !right) continue;
    const transfer =
      ARB_K * (left.compression - right.compression) +
      ARB_C * (left.closingSpeed - right.closingSpeed);
    left.force += transfer;
    right.force -= transfer;
    for (const s of [left, right]) {
      if (s.force > s.coil.forceCap) s.force = s.coil.forceCap;
      if (s.force < -s.coil.forceCap * 0.35) s.force = -s.coil.forceCap * 0.35;
    }
  }

  for (const s of samples) {
    const impulse = s.force * dt;
    chassis.applyImpulseAtPoint({ x: ux * impulse, y: uy * impulse, z: uz * impulse }, s.p0, true);
    s.axle.body.applyImpulseAtPoint({ x: -ux * impulse, y: -uy * impulse, z: -uz * impulse }, s.p1, true);
  }
}

