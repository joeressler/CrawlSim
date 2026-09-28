/**
 * Phase 3 coilovers between chassis/axle shock mounts.
 * restLength = geometric mount distance at build (BOM-true; no rest bias fighting links).
 * Weight preload = mg/n so hang holds at mount geometry without fake rest stretch.
 * Forces along world-up (chassis-up coupled into roll on soft sphericals).
 * Mild L/R anti-roll restores roll stiffness without pumping idle chatter.
 * Compress spring + preload; extension keeps tapered preload + light damper only.
 * Hang floor + hard bump stop: chassis must not fold through axle on bumps
 * (spherical impulse joints stretch; extension path alone cannot restore hang).
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
  /** Static support share at mount rest (N). Keeps hang without rest-length bias. */
  preload: number;
  /** Chassis COM Y - axle COM Y at mount-true build (m). */
  restHang: number;
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
  const cy = chassis.translation().y;
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
    // Impact budget: ~1.5x vehicle weight per corner so bump stops are not ride-capped.
    const bumpForceCap = Math.max(forceCap * 6, (2.5 * chassisMass * 9.81) / n);
    const restHang = Math.max(0.02, cy - axle.body.translation().y);
    out.push({
      def: shock,
      restLength,
      maxTravel,
      springK,
      damperC,
      forceCap,
      preload,
      restHang,
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

  // World-up: chassis-up couples into roll when soft sphericals let rails twist
  // on planted hubs. Mild ARB restores roll stiffness without idle chatter.
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
      force = c.springK * compression + c.preload + c.damperC * closingSpeed;
      const bumpDepth = compression - c.maxTravel;
      if (bumpDepth > 0) {
        // Hard packer: stiff + not clipped by ride forceCap (was the crumple leak).
        const bump =
          c.springK * 18 * bumpDepth +
          c.springK * 260 * bumpDepth * bumpDepth +
          c.damperC * 4 * Math.max(0, closingSpeed);
        force += bump;
      }
    } else {
      // Extension: taper preload so gravity can settle; no spring pull fighting links.
      const taper = Math.max(0, 1 + compression / Math.max(1e-3, c.restLength * 0.25));
      force = c.preload * taper + c.damperC * 0.35 * closingSpeed;
    }
    // Ride forces stay soft-capped; bump-stop may use the higher bump budget.
    const cap = compression > c.maxTravel ? c.bumpForceCap : c.forceCap;
    if (force > cap) force = cap;
    if (force < -c.forceCap * 0.35) force = -c.forceCap * 0.35;
    samples.push({ coil: c, axle, p0, p1, compression, closingSpeed, force });
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

  for (const s of samples) {
    const impulse = s.force * dt;
    chassis.applyImpulseAtPoint({ x: ux * impulse, y: uy * impulse, z: uz * impulse }, s.p0, true);
    s.axle.body.applyImpulseAtPoint({ x: -ux * impulse, y: -uy * impulse, z: -uz * impulse }, s.p1, true);
  }

  // Hang floor: when axle hops above the rails (shock often EXTENDS while hang
  // collapses), ride springs do nothing useful. Soft spring on COM hang restores
  // separation without Rapier lock joints. Inactive near mount-true rest.
  applyHangFloors(chassis, axles, coilovers, dt);
}

/**
 * Crumple floor on chassis-up COM hang - ONLY past full mechanical travel.
 * Normal compress to (restHang - maxTravel) is owned by coilovers/bump-stop.
 * Firing inside that band double-springs the kit and flips axle-ram impacts.
 * Activates when hang drops below ~1cm (rails folding onto/through the axle).
 */
function applyHangFloors(
  chassis: RAPIER.RigidBody,
  axles: Map<string, KitAxleRuntime>,
  coilovers: CoiloverRuntime[],
  dt: number
): void {
  const seen = new Set<string>();
  const cr = chassis.rotation();
  scratchQ.set(cr.x, cr.y, cr.z, cr.w);
  scratchV.set(0, 1, 0).applyQuaternion(scratchQ);
  const upx = scratchV.x;
  const upy = scratchV.y;
  const upz = scratchV.z;
  // Ignore when inverted / tumbling - do not fight a tip-over.
  if (upy < 0.45) return;

  for (const coil of coilovers) {
    if (seen.has(coil.axleId)) continue;
    seen.add(coil.axleId);
    const axle = axles.get(coil.axleId);
    if (!axle) continue;

    const ct = chassis.translation();
    const cv = chassis.linvel();
    const at = axle.body.translation();
    const av = axle.body.linvel();
    const hang = (ct.x - at.x) * upx + (ct.y - at.y) * upy + (ct.z - at.z) * upz;

    // Past bump travel: restHang - maxTravel is still legal. Floor sits below it.
    const travelFloor = coil.restHang - coil.maxTravel;
    const softFloor = Math.min(0.012, Math.max(0.006, travelFloor - 0.008));
    if (hang >= softFloor) continue;

    const closing =
      (av.x - cv.x) * upx + (av.y - cv.y) * upy + (av.z - cv.z) * upz;
    const softErr = softFloor - hang;
    let force = 1100 * softErr + 48 * Math.max(0, closing);

    // Hard packer for zero / negative hang (chassis through axle).
    if (hang < 0.004) {
      const hardErr = 0.004 - hang;
      force += 5000 * hardErr + 22000 * hardErr * hardErr + 80 * Math.max(0, closing);
    }

    const cap = Math.max(coil.bumpForceCap * 2.5, 150);
    if (force > cap) force = cap;
    if (force < 0) force = 0;

    const impulse = force * dt;
    chassis.applyImpulse({ x: upx * impulse, y: upy * impulse, z: upz * impulse }, true);
    axle.body.applyImpulse({ x: -upx * impulse, y: -upy * impulse, z: -upz * impulse }, true);
  }
}
