/**
 * Phase 3 coilovers between chassis/axle shock mounts.
 * restLength = geometric mount distance at build (BOM-true; no rest bias fighting links).
 * Weight preload = mg/n so hang holds at mount geometry without fake rest stretch.
 * Forces along shock mount axis (same metric as compression = restLength - dist).
 * Mild L/R anti-roll restores roll stiffness without pumping idle chatter.
 * Compress spring + preload; extension keeps tapered preload + light damper only.
 * Shock-axis bump packer uses a higher force budget than ride forceCap (no COM hang
 * floor — those impulses fought links and caused throttle pitch-dive/axle drag).
 * Hub spheres = only plant; no Rapier spring joints. Crumple stop = chassis↔axle collision.
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
  /** Static support share at mount rest (N). Keeps hang without rest-length bias. */
  preload: number;
  /** Impact bump-stop force budget (N); higher than ride forceCap. */
  bumpForceCap: number;
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
  const n = Math.max(1, kit.shocks.length);
  // Full weight share at mount rest - hang without stretching restLength past BOM.
  const preload = (chassisMass * 9.81) / n;
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
    // Hard gate: rest length equals mounts (no F/R hang bias fighting 4-link + panhard).
    const restLength = Math.max(0.04, Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z));
    const springK = shock.springK;
    const mEff = Math.max(0.5, chassisMass / n);
    const critical = 2 * Math.sqrt(springK * mEff);
    const damperC = Math.max(shock.damperC, critical * 2.4);
    const maxTravel = Math.min(shock.maxTravel, restLength * 0.4);
    const forceCap = (2.8 * chassisMass * 9.81) / n;
    // Shock-axis packer only — not clipped by soft ride cap.
    const bumpForceCap = Math.max(forceCap * 4, (1.8 * chassisMass * 9.81) / n);
    out.push({
      def: shock,
      restLength,
      maxTravel,
      springK,
      damperC,
      forceCap,
      preload,
      bumpForceCap,
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

  // Metric and delivery share the shock axis (mount-to-mount). Chassis-up-only
  // delivery was tried to break coils+drive pitch coupling; it killed climb/upright.
  type Sample = {
    coil: CoiloverRuntime;
    axle: KitAxleRuntime;
    p0: { x: number; y: number; z: number };
    p1: { x: number; y: number; z: number };
    ux: number;
    uy: number;
    uz: number;
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
    const dx = p0.x - p1.x;
    const dy = p0.y - p1.y;
    const dz = p0.z - p1.z;
    const dist = Math.hypot(dx, dy, dz);
    if (dist < 1e-5) continue;
    const ux = dx / dist;
    const uy = dy / dist;
    const uz = dz / dist;
    const compression = c.restLength - dist;

    const v0 = chassis.velocityAtPoint(p0);
    const v1 = axle.body.velocityAtPoint(p1);
    const closingSpeed = (v1.x - v0.x) * ux + (v1.y - v0.y) * uy + (v1.z - v0.z) * uz;

    let force = 0;
    if (compression > 0) {
      force = c.springK * compression + c.preload + c.damperC * closingSpeed;
      const bumpDepth = compression - c.maxTravel;
      if (bumpDepth > 0) {
        const bump =
          c.springK * 14 * bumpDepth +
          c.springK * 180 * bumpDepth * bumpDepth +
          c.damperC * 3 * Math.max(0, closingSpeed);
        force += bump;
      }
    } else {
      const taper = Math.max(0, 1 + compression / Math.max(1e-3, c.restLength * 0.25));
      force = c.preload * taper + c.damperC * 0.35 * closingSpeed;
    }
    const cap = compression > c.maxTravel ? c.bumpForceCap : c.forceCap;
    if (force > cap) force = cap;
    if (force < -c.forceCap * 0.35) force = -c.forceCap * 0.35;
    samples.push({
      coil: c,
      axle,
      p0,
      p1,
      ux,
      uy,
      uz,
      compression,
      closingSpeed,
      force,
    });
  }

  const byAxle = new Map<string, Sample[]>();
  for (const s of samples) {
    const list = byAxle.get(s.coil.axleId) ?? [];
    list.push(s);
    byAxle.set(s.coil.axleId, list);
  }
  // Softened vs prior 55/40: strong ARB + preload locked a pitched idle and pumped Vy.
  const ARB_K = 28;
  const ARB_C = 12;
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
      const cap = s.compression > s.coil.maxTravel ? s.coil.bumpForceCap : s.coil.forceCap;
      if (s.force > cap) s.force = cap;
      if (s.force < -s.coil.forceCap * 0.35) s.force = -s.coil.forceCap * 0.35;
    }
  }

  // Chassis-up projection of shock-axis force (Newton pair). Full shock-axis
  // seeds coils+drive axle tumble (foldDiag A ~step 15). Climb uses hub Coulomb
  // / face boost — not shock fore-aft.
  const cr = chassis.rotation();
  scratchQ.set(cr.x, cr.y, cr.z, cr.w);
  scratchV.set(0, 1, 0).applyQuaternion(scratchQ);
  const upLen = scratchV.length();
  const upx = upLen > 1e-8 ? scratchV.x / upLen : 0;
  const upy = upLen > 1e-8 ? scratchV.y / upLen : 1;
  const upz = upLen > 1e-8 ? scratchV.z / upLen : 0;

  for (const s of samples) {
    const impulse = s.force * dt;
    const along = s.ux * upx + s.uy * upy + s.uz * upz;
    // Always chassis-up: full shock-axis under throttle re-seeds axle fold
    // (foldDiag A). Lip climb is hub Coulomb / face boost, not shock fore-aft.
    const ux = upx * along;
    const uy = upy * along;
    const uz = upz * along;
    chassis.applyImpulseAtPoint({ x: ux * impulse, y: uy * impulse, z: uz * impulse }, s.p0, true);
    s.axle.body.applyImpulseAtPoint({ x: -ux * impulse, y: -uy * impulse, z: -uz * impulse }, s.p1, true);
  }
}
