/**
 * Unilateral hard fold/pitch stop for kit axles (impulse/torque only — no setTranslation).
 *
 * Spherical/revolute links own locate. Fires only past the travel envelope.
 * Must run AFTER hub-drive.
 */
import type RAPIER from "@dimforge/rapier3d-compat";
import * as THREE from "three";
import type { KitAxleRuntime } from "./kitBodies.ts";

const scratchQ = new THREE.Quaternion();
const scratchInv = new THREE.Quaternion();
const scratchLocal = new THREE.Vector3();
const scratchFwd = new THREE.Vector3();
const scratchUp = new THREE.Vector3();
const scratchLat = new THREE.Vector3();

/** Chassis-local Z past which front axle is folded under (rest ≈ -0.1565). */
const FRONT_FOLD_LIMIT_Z = -0.11;
/** Chassis-local Z past which rear axle is folded under (rest ≈ +0.1565). */
const REAR_FOLD_LIMIT_Z = 0.11;
const FOLD_K = 4200;
const FOLD_C = 160;
const FOLD_FORCE_CAP = 480;

/** Bumper past wrap. */
const PITCH_SOFT = 0.13;
const PITCH_K = 280;
const PITCH_C = 45;
const PITCH_TORQUE_CAP = 45;
const PITCH_RATE_C = 7;
const PITCH_RATE_ARM = 0.16;

function clamp(v: number, lo: number, hi: number): number {
  return Math.min(hi, Math.max(lo, v));
}

export function applyHardFoldStop(
  chassis: RAPIER.RigidBody,
  axles: Map<string, KitAxleRuntime>,
  dt: number
): void {
  if (!(dt > 0)) return;
  const cr = chassis.rotation();
  scratchQ.set(cr.x, cr.y, cr.z, cr.w);
  scratchInv.copy(scratchQ).invert();
  scratchUp.set(0, 1, 0).applyQuaternion(scratchQ);
  // Stay armed through steep climb attitudes. The old guard disabled the fold stop
  // while the chassis was still only pitched over, which let the axle fold under
  // and the rig roll inverted on ramp entries.
  if (scratchUp.y < -0.2) return;

  scratchFwd.set(0, 0, -1).applyQuaternion(scratchQ);
  const upDot = scratchFwd.dot(scratchUp);
  scratchFwd.addScaledVector(scratchUp, -upDot);
  if (scratchFwd.lengthSq() < 1e-8) return;
  scratchFwd.normalize();

  scratchLat.set(1, 0, 0).applyQuaternion(scratchQ);

  const ct = chassis.translation();
  const cv = chassis.linvel();

  for (const axle of axles.values()) {
    const at = axle.body.translation();
    scratchLocal.set(at.x - ct.x, at.y - ct.y, at.z - ct.z).applyQuaternion(scratchInv);
    const localZ = scratchLocal.z;

    let excess = 0;
    let sign = 0;
    if (axle.id === "front") {
      if (localZ > FRONT_FOLD_LIMIT_Z) {
        excess = localZ - FRONT_FOLD_LIMIT_Z;
        sign = 1;
      }
    } else if (axle.id === "rear") {
      if (localZ < REAR_FOLD_LIMIT_Z) {
        excess = REAR_FOLD_LIMIT_Z - localZ;
        sign = -1;
      }
    }

    if (excess > 0 && sign !== 0) {
      const av = axle.body.linvel();
      const ux = scratchFwd.x * sign;
      const uy = scratchFwd.y * sign;
      const uz = scratchFwd.z * sign;
      const closing = (av.x - cv.x) * ux + (av.y - cv.y) * uy + (av.z - cv.z) * uz;
      let force = FOLD_K * excess + FOLD_C * Math.max(0, closing);
      force = clamp(force, 0, FOLD_FORCE_CAP);
      const impulse = force * dt;
      axle.body.applyImpulse({ x: ux * impulse, y: uy * impulse, z: uz * impulse }, true);
      // Partial chassis share keeps links from stretching without fully canceling drive.
      // Partial chassis share keeps links from stretching without fully canceling drive.
      const share = 0.13;
      chassis.applyImpulse(
        { x: -ux * impulse * share, y: -uy * impulse * share, z: -uz * impulse * share },
        true
      );
    }

    const ar = axle.body.rotation();
    const aq = new THREE.Quaternion(ar.x, ar.y, ar.z, ar.w);
    const rel = scratchQ.clone().invert().multiply(aq);
    const e = new THREE.Euler().setFromQuaternion(rel, "YXZ");
    const pitch = e.x;
    const absPitch = Math.abs(pitch);
    const ang = axle.body.angvel();
    const omega = ang.x * scratchLat.x + ang.y * scratchLat.y + ang.z * scratchLat.z;
    // Only while translating — idle articulation must not fight rate bleed.
    const moving = Math.hypot(cv.x, cv.z) > 0.4;
    if (moving && absPitch > PITCH_RATE_ARM) {
      let rateTorque = -PITCH_RATE_C * omega;
      rateTorque = clamp(rateTorque, -PITCH_TORQUE_CAP * 0.35, PITCH_TORQUE_CAP * 0.35);
      const jr = rateTorque * dt;
      axle.body.applyTorqueImpulse(
        { x: scratchLat.x * jr, y: scratchLat.y * jr, z: scratchLat.z * jr },
        true
      );
    }
    if (absPitch > PITCH_SOFT) {
      const excessPitch = absPitch - PITCH_SOFT;
      const psign = pitch >= 0 ? 1 : -1;
      let torque = -psign * PITCH_K * excessPitch - PITCH_C * omega;
      torque = clamp(torque, -PITCH_TORQUE_CAP, PITCH_TORQUE_CAP);
      const j = torque * dt;
      axle.body.applyTorqueImpulse(
        { x: scratchLat.x * j, y: scratchLat.y * j, z: scratchLat.z * j },
        true
      );
    }
  }
}

/**
 * Pose projection after the physics step.
 * A rounded ramp nose still lets the housing walk a centimetre, and that is
 * enough to shorten or lengthen the short upper arms past the fold gate.
 * Straight driving only — steering needs the axle free to swing.
 */
const STATION_HALF = 0.002;
const PITCH_PROJECT = 0.05;
const DROOP_PROJECT = 0.008;

export function projectAxleStation(
  chassis: RAPIER.RigidBody,
  axles: Map<string, KitAxleRuntime>
): void {
  const cr = chassis.rotation();
  scratchQ.set(cr.x, cr.y, cr.z, cr.w);
  scratchInv.copy(scratchQ).invert();
  const ct = chassis.translation();
  for (const axle of axles.values()) {
    const at = axle.body.translation();
    scratchLocal.set(at.x - ct.x, at.y - ct.y, at.z - ct.z).applyQuaternion(scratchInv);
    const ar = axle.body.rotation();
    const aq = new THREE.Quaternion(ar.x, ar.y, ar.z, ar.w);
    const rel = scratchQ.clone().invert().multiply(aq);
    const euler = new THREE.Euler(0, 0, 0, "YXZ").setFromQuaternion(rel);
    let moved = false;
    if (scratchLocal.y < axle.restLocal.y - DROOP_PROJECT) {
      scratchLocal.y = axle.restLocal.y - DROOP_PROJECT;
      moved = true;
    }
    if (Math.abs(euler.x) > PITCH_PROJECT) {
      euler.x = Math.sign(euler.x) * PITCH_PROJECT;
      rel.setFromEuler(euler);
      const world = scratchQ.clone().multiply(rel);
      axle.body.setRotation({ x: world.x, y: world.y, z: world.z, w: world.w }, true);
      moved = true;
    }
    const zLo = axle.restLocal.z - STATION_HALF;
    const zHi = axle.restLocal.z + STATION_HALF;
    if (scratchLocal.z < zLo) {
      scratchLocal.z = zLo;
      moved = true;
    } else if (scratchLocal.z > zHi) {
      scratchLocal.z = zHi;
      moved = true;
    }
    if (!moved) continue;
    scratchFwd.copy(scratchLocal).applyQuaternion(scratchQ);
    axle.body.setTranslation(
      { x: ct.x + scratchFwd.x, y: ct.y + scratchFwd.y, z: ct.z + scratchFwd.z },
      true
    );
  }
}

/**
 * Keep the front axle's chassis-local Z near rest even while steering.
 * Full projectAxleStation is skipped under steer so the housing can yaw;
 * without this the steer axle rubberbands fore-aft on every turn.
 */
export function projectFrontStation(
  chassis: RAPIER.RigidBody,
  axles: Map<string, KitAxleRuntime>
): void {
  const front = axles.get("front");
  if (!front) return;
  const cr = chassis.rotation();
  scratchQ.set(cr.x, cr.y, cr.z, cr.w);
  scratchInv.copy(scratchQ).invert();
  const ct = chassis.translation();
  const at = front.body.translation();
  scratchLocal.set(at.x - ct.x, at.y - ct.y, at.z - ct.z).applyQuaternion(scratchInv);
  const half = 0.0015;
  const zLo = front.restLocal.z - half;
  const zHi = front.restLocal.z + half;
  if (scratchLocal.z >= zLo && scratchLocal.z <= zHi) return;
  scratchLocal.z = clamp(scratchLocal.z, zLo, zHi);
  scratchFwd.copy(scratchLocal).applyQuaternion(scratchQ);
  front.body.setTranslation(
    { x: ct.x + scratchFwd.x, y: ct.y + scratchFwd.y, z: ct.z + scratchFwd.z },
    true
  );
}

/**
 * Soft fore-aft station spring. Distance rods allow the housing to walk rearward
 * under throttle; this holds chassis-local Z near the rest pose.
 */
export function holdAxleStation(
  chassis: RAPIER.RigidBody,
  axles: Map<string, KitAxleRuntime>,
  dt: number
): void {
  if (!(dt > 0)) return;
  const cr = chassis.rotation();
  scratchQ.set(cr.x, cr.y, cr.z, cr.w);
  scratchInv.copy(scratchQ).invert();
  scratchFwd.set(0, 0, 1).applyQuaternion(scratchQ);
  const ct = chassis.translation();
  for (const axle of axles.values()) {
    const at = axle.body.translation();
    scratchLocal.set(at.x - ct.x, at.y - ct.y, at.z - ct.z).applyQuaternion(scratchInv);
    const err = scratchLocal.z - axle.restLocal.z;
    // Front bites earlier — less free rubberband before the station spring holds.
    const deadband = axle.id === "front" ? 0.010 : 0.015;
    if (Math.abs(err) < deadband) continue;
    const force = clamp(-err * 1800, -210, 210);
    const j = force * dt;
    axle.body.applyImpulse(
      { x: scratchFwd.x * j, y: scratchFwd.y * j, z: scratchFwd.z * j },
      true
    );
    chassis.applyImpulse(
      { x: -scratchFwd.x * j * 0.4, y: -scratchFwd.y * j * 0.4, z: -scratchFwd.z * j * 0.4 },
      true
    );
  }
}
