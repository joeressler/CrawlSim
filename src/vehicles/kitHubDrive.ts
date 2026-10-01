/**
 * Hub Coulomb grip: contact + drive at axle hubs.
 * Soft hub spheres (friction 0) = normal plant only. Coilovers = ride height.
 * Spherical joints = axle locate. Coulomb owns long/lat grip (no Rapier hub μ).
 * Planar COM motor is airborne-only; planted drive is Coulomb + face climb.
 * Sharp lips: climb tangent blends world-up-on-face; tire pull delivers at the
 * contact so the hub claws up instead of shoving into the wall.
 * Mild pitch-restore / reverse-yaw are chassis attitude only.
 */
import RAPIER from "@dimforge/rapier3d-compat";
import * as THREE from "three";
import type { DriveInput } from "../input/Input.ts";
import type { KitWheelDef, RigDef } from "./types.ts";
import type { KitAxleRuntime } from "./kitBodies.ts";
import { hubWorldPosition } from "./kitBodies.ts";
import { kitDiagFlags } from "./kitDiagFlags.ts";

const FRICTION_ITERATIONS = 7;
const FRICTION_RELAX = 0.62;
const ROLL_KEEP = 0.18;
/** Mild axle yaw vs chassis (torque only — no setRotation / angvel snap). */
const AXLE_YAW_K = 18;
const AXLE_YAW_C = 1.6;
const AXLE_YAW_TORQUE_CAP = 4.5;
const AXLE_YAW_SOFT = 0.12;

export type HubDriveState = {
  commandSpeed: number;
  steer: number;
  /** Seconds to suppress face climb after a high-speed face impact. */
  impactHold: number;
};

export function createHubDriveState(): HubDriveState {
  return { commandSpeed: 0, steer: 0, impactHold: 0 };
}

export function resetHubDriveState(state: HubDriveState): void {
  state.commandSpeed = 0;
  state.steer = 0;
  state.impactHold = 0;
}

type HubWheel = {
  def: KitWheelDef;
  axle: KitAxleRuntime;
  /** Lateral sign for roll authority (from hub X). */
  sideX: number;
  steered: boolean;
  driven: boolean;
  radius: number;
  mu: number;
};

type HubContact = {
  wheel: HubWheel;
  fn: number;
  px: number;
  py: number;
  pz: number;
  nx: number;
  ny: number;
  nz: number;
  tx: number;
  ty: number;
  tz: number;
  cx: number;
  cy: number;
  cz: number;
  sx: number;
  sy: number;
  sz: number;
  invLong: number;
  invChassis: number;
  invLat: number;
  accLong: number;
  accLat: number;
  grip: number;
  gripIn: number;
  /** Steep face / lip plant — climb tangent biased up-the-face. */
  face: boolean;
};

const q = new THREE.Quaternion();
const basis = new THREE.Vector3();
const basisUp = new THREE.Vector3();
const scratchForward = new THREE.Vector3();
const scratchRight = new THREE.Vector3();
const scratchQuat = new THREE.Quaternion();
const scratchLat = new THREE.Vector3();

function clampImpulse(value: number, min: number, max: number): number {
  if (!Number.isFinite(value)) return 0;
  if (max < min) return 0;
  return Math.min(max, Math.max(min, value));
}

function gripFromSupport(support: number, minNormalY: number): number {
  const full = Math.max(minNormalY + 0.35, minNormalY + 1e-3);
  return clampImpulse((support - minNormalY) / (full - minNormalY), 0, 1);
}

function invMassAlong(
  chassis: RAPIER.RigidBody,
  px: number,
  py: number,
  pz: number,
  ux: number,
  uy: number,
  uz: number
): number {
  const com = chassis.worldCom();
  const rx = px - com.x;
  const ry = py - com.y;
  const rz = pz - com.z;
  const cx = ry * uz - rz * uy;
  const cy = rz * ux - rx * uz;
  const cz = rx * uy - ry * ux;
  const invI = chassis.effectiveWorldInvInertia();
  const e = invI.elements;
  const ix = e[0] * cx + e[1] * cy + e[2] * cz;
  const iy = e[1] * cx + e[3] * cy + e[4] * cz;
  const iz = e[2] * cx + e[4] * cy + e[5] * cz;
  const inv = chassis.invMass() + cx * ix + cy * iy + cz * iz;
  return inv > 1e-8 ? inv : 1e-8;
}

function yawInvInertia(chassis: RAPIER.RigidBody, ux: number, uy: number, uz: number): number {
  const invI = chassis.effectiveWorldInvInertia();
  const e = invI.elements;
  const ix = e[0] * ux + e[1] * uy + e[2] * uz;
  const iy = e[1] * ux + e[3] * uy + e[4] * uz;
  const iz = e[2] * ux + e[4] * uy + e[5] * uz;
  const inv = ux * ix + uy * iy + uz * iz;
  return inv > 1e-8 ? inv : 1e-8;
}

function castHubSupport(
  world: RAPIER.World,
  hub: THREE.Vector3,
  radius: number,
  axleBody: RAPIER.RigidBody,
  gx: number,
  gy: number,
  gz: number,
  minNormalY: number
): { toi: number; nx: number; ny: number; nz: number; support: number } | null {
  const gMag = Math.hypot(gx, gy, gz) || 9.81;
  const downx = gx / gMag;
  const downy = gy / gMag;
  const downz = gz / gMag;
  const ray = new RAPIER.Ray(
    { x: hub.x, y: hub.y, z: hub.z },
    { x: downx, y: downy, z: downz }
  );
  const hit = world.castRayAndGetNormal(ray, radius * 1.45, false, undefined, undefined, undefined, axleBody);
  if (!hit) return null;
  let nx = hit.normal.x;
  let ny = hit.normal.y;
  let nz = hit.normal.z;
  const nLen = Math.hypot(nx, ny, nz);
  if (nLen < 1e-6) return null;
  nx /= nLen;
  ny /= nLen;
  nz /= nLen;
  if (nx * downx + ny * downy + nz * downz > 0) {
    nx = -nx;
    ny = -ny;
    nz = -nz;
  }
  const support = -(nx * gx + ny * gy + nz * gz) / gMag;
  if (support < minNormalY) return null;
  if (hit.timeOfImpact > radius * 1.2) return null;
  return { toi: hit.timeOfImpact, nx, ny, nz, support };
}


/**
 * Forward face probe for vertical-face / lip traction when the down-ray rejects
 * steep normals, or a wall sits ahead of the tread.
 */
function castHubFace(
  world: RAPIER.World,
  hub: THREE.Vector3,
  radius: number,
  axleBody: RAPIER.RigidBody,
  fx: number,
  fy: number,
  fz: number,
  gx: number,
  gy: number,
  gz: number,
  /** Extra reach for axle-straddle lips (fronts on deck, rears still low). */
  reachMul = 1
): { toi: number; nx: number; ny: number; nz: number; support: number } | null {
  const fLen = Math.hypot(fx, fy, fz);
  if (fLen < 1e-6) return null;
  const ux = fx / fLen;
  const uy = fy / fLen;
  const uz = fz / fLen;
  const ray = new RAPIER.Ray(
    { x: hub.x, y: hub.y, z: hub.z },
    { x: ux, y: uy, z: uz }
  );
  const castLen = radius * 1.85 * reachMul;
  const acceptLen = radius * 1.55 * reachMul;
  const hit = world.castRayAndGetNormal(ray, castLen, false, undefined, undefined, undefined, axleBody);
  if (!hit) return null;
  if (hit.timeOfImpact > acceptLen) return null;
  let nx = hit.normal.x;
  let ny = hit.normal.y;
  let nz = hit.normal.z;
  const nLen = Math.hypot(nx, ny, nz);
  if (nLen < 1e-6) return null;
  nx /= nLen;
  ny /= nLen;
  nz /= nLen;
  if (nx * ux + ny * uy + nz * uz > 0) {
    nx = -nx;
    ny = -ny;
    nz = -nz;
  }
  const gMag = Math.hypot(gx, gy, gz) || 9.81;
  const support = -(nx * gx + ny * gy + nz * gz) / gMag;
  if (support < -0.15) return null;
  return { toi: hit.timeOfImpact, nx, ny, nz, support: Math.max(0, support) };
}

/**
 * Drive tangent on a face/lip. Straight-into-wall collapses chassis-forward onto
 * the normal — blend world-up-on-face so tires claw toward the crest.
 */
function faceClimbTangent(
  nx: number,
  ny: number,
  nz: number,
  fx: number,
  fy: number,
  fz: number,
  upx: number,
  upy: number,
  upz: number
): { tx: number; ty: number; tz: number } | null {
  const intoF = fx * nx + fy * ny + fz * nz;
  let ffx = fx - nx * intoF;
  let ffy = fy - ny * intoF;
  let ffz = fz - nz * intoF;
  const fLen = Math.hypot(ffx, ffy, ffz);

  const intoU = upx * nx + upy * ny + upz * nz;
  let uux = upx - nx * intoU;
  let uuy = upy - ny * intoU;
  let uuz = upz - nz * intoU;
  let uLen = Math.hypot(uux, uuy, uuz);
  if (uLen > 1e-5) {
    uux /= uLen;
    uuy /= uLen;
    uuz /= uLen;
    // Crest direction: positive world-up along the face.
    if (uux * upx + uuy * upy + uuz * upz < 0) {
      uux = -uux;
      uuy = -uuy;
      uuz = -uuz;
    }
  } else {
    uLen = 0;
  }

  // Steeper face (low ny) → more up-the-face claw.
  const steep = clampImpulse(1 - Math.max(0, ny), 0, 1);
  const upW = 0.35 + 0.6 * steep;

  let tx: number;
  let ty: number;
  let tz: number;
  if (fLen < 1e-5 && uLen < 1e-5) return null;
  if (fLen < 1e-5) {
    tx = uux;
    ty = uuy;
    tz = uuz;
  } else {
    ffx /= fLen;
    ffy /= fLen;
    ffz /= fLen;
    if (uLen < 1e-5) {
      tx = ffx;
      ty = ffy;
      tz = ffz;
    } else {
      const fw = 1 - upW;
      tx = ffx * fw + uux * upW;
      ty = ffy * fw + uuy * upW;
      tz = ffz * fw + uuz * upW;
    }
  }
  const tLen = Math.hypot(tx, ty, tz);
  if (tLen < 1e-5) return null;
  return { tx: tx / tLen, ty: ty / tLen, tz: tz / tLen };
}

function frictionLimit(contact: HubContact, maxForce: number): number {
  return Math.min(contact.wheel.mu * contact.grip * contact.fn, maxForce);
}

function tangentVelocity(
  chassis: RAPIER.RigidBody,
  contact: HubContact,
  gx: number,
  gy: number,
  gz: number,
  dt: number
): { vLong: number; vLat: number; vChassis: number } {
  const vel = chassis.velocityAtPoint({ x: contact.px, y: contact.py, z: contact.pz });
  const vx = vel.x + gx * dt;
  const vy = vel.y + gy * dt;
  const vz = vel.z + gz * dt;
  const vn = vx * contact.nx + vy * contact.ny + vz * contact.nz;
  const tpx = vx - contact.nx * vn;
  const tpy = vy - contact.ny * vn;
  const tpz = vz - contact.nz * vn;
  return {
    vLong: tpx * contact.tx + tpy * contact.ty + tpz * contact.tz,
    vLat: tpx * contact.sx + tpy * contact.sy + tpz * contact.sz,
    vChassis: tpx * contact.cx + tpy * contact.cy + tpz * contact.cz,
  };
}


function applyAxisImpulse(
  chassis: RAPIER.RigidBody,
  contact: HubContact,
  ax: number,
  ay: number,
  az: number,
  impulse: number
): void {
  if (impulse === 0) return;
  // Flat plant: COM delivery (hub-point wheelies). Near-vertical face: at contact
  // so the tire claws up. Milder faces stay COM to avoid crest tip-over.
  if (contact.face && contact.ny < 0.4) {
    chassis.applyImpulseAtPoint(
      { x: ax * impulse, y: ay * impulse, z: az * impulse },
      { x: contact.px, y: contact.py, z: contact.pz },
      true
    );
    return;
  }
  chassis.applyImpulse({ x: ax * impulse, y: ay * impulse, z: az * impulse }, true);
}

function applyLateral(
  chassis: RAPIER.RigidBody,
  contact: HubContact,
  ax: number,
  ay: number,
  az: number,
  impulse: number
): void {
  if (impulse === 0) return;
  const jx = ax * impulse;
  const jy = ay * impulse;
  const jz = az * impulse;
  chassis.applyImpulseAtPoint({ x: jx, y: jy, z: jz }, { x: contact.px, y: contact.py, z: contact.pz }, true);
  const com = chassis.worldCom();
  const rx = contact.px - com.x;
  const ry = contact.py - com.y;
  const rz = contact.pz - com.z;
  const tx = ry * jz - rz * jy;
  const ty = rz * jx - rx * jz;
  const tz = rx * jy - ry * jx;
  const rot = chassis.rotation();
  scratchQuat.set(rot.x, rot.y, rot.z, rot.w);
  scratchForward.set(0, 0, -1).applyQuaternion(scratchQuat);
  const roll = tx * scratchForward.x + ty * scratchForward.y + tz * scratchForward.z;
  const cancel = roll * (1 - ROLL_KEEP);
  chassis.applyTorqueImpulse(
    { x: -scratchForward.x * cancel, y: -scratchForward.y * cancel, z: -scratchForward.z * cancel },
    true
  );
}

function rollAuthority(contacts: HubContact[], weight: number): number {
  if (contacts.length < 3) return 0;
  let leftFn = 0;
  let rightFn = 0;
  for (const contact of contacts) {
    if (contact.wheel.sideX <= 0) leftFn += contact.fn;
    else rightFn += contact.fn;
  }
  const sidePlant = 0.18 * weight;
  return clampImpulse(Math.min(leftFn, rightFn) / sidePlant, 0, 1);
}

function applySteer(
  chassis: RAPIER.RigidBody,
  contacts: HubContact[],
  maxForce: number,
  steerInput: number,
  dt: number,
  rollScale: number
): void {
  if (steerInput === 0 || rollScale <= 0) return;
  for (const contact of contacts) {
    if (!contact.wheel.steered) continue;
    const cap = frictionLimit(contact, maxForce) * dt * rollScale * contact.gripIn;
    const used = Math.hypot(contact.accLong, contact.accLat);
    const room = Math.sqrt(Math.max(0, cap * cap - used * used));
    const jLat = clampImpulse(steerInput * cap * 0.62, -room, room);
    applyLateral(chassis, contact, contact.sx, contact.sy, contact.sz, jLat);
    contact.accLat += jLat;
  }
}

/** Reverse: yaw about up so the tail swings instead of scrubbing steered tires. */
function applyReverseYaw(
  chassis: RAPIER.RigidBody,
  steerAngle: number,
  steerInput: number,
  fwdSpeed: number,
  wheelbase: number,
  gx: number,
  gy: number,
  gz: number,
  gMag: number,
  rollScale: number
): void {
  if (steerInput === 0 || rollScale <= 0) return;
  const delta = steerInput * steerAngle;
  const speed = Math.min(fwdSpeed, -0.4);
  const omegaTarget = (speed / Math.max(0.2, wheelbase)) * Math.tan(delta);
  const ang = chassis.angvel();
  const upx = -gx / gMag;
  const upy = -gy / gMag;
  const upz = -gz / gMag;
  const omega = ang.x * upx + ang.y * upy + ang.z * upz;
  const inv = yawInvInertia(chassis, upx, upy, upz);
  const correct = clampImpulse(((omegaTarget - omega) * 0.28) / inv, -0.22, 0.22) * rollScale;
  if (correct === 0) return;
  chassis.applyTorqueImpulse({ x: upx * correct, y: upy * correct, z: upz * correct }, true);
}

function solveContact(
  chassis: RAPIER.RigidBody,
  contact: HubContact,
  maxForce: number,
  commandSpeed: number,
  steerInput: number,
  motorCap: number,
  rollScale: number,
  reversing: boolean,
  gx: number,
  gy: number,
  gz: number,
  dt: number
): void {
  const { wheel } = contact;
  const slip = tangentVelocity(chassis, contact, gx, gy, gz, dt);
  const steerShare = wheel.steered && steerInput !== 0 && !reversing ? 0.35 : 0;
  const maxFric = frictionLimit(contact, maxForce) * dt * (1 - steerShare);

  // Longitudinal along contact tangent (tx includes uphill on ramps/faces).
  // Face tx is travel-aligned when reversing — chase abs speed along that climb dir.
  const driveStraight = reversing && wheel.steered;
  const vDrive = driveStraight ? slip.vChassis : slip.vLong;
  const targetLong = wheel.driven
    ? contact.face && reversing
      ? Math.abs(commandSpeed)
      : commandSpeed
    : vDrive;
  const driveInv = driveStraight ? contact.invChassis : contact.invLong;
  const motor = Math.min(motorCap, maxFric);
  const wantLong = clampImpulse(
    (-(vDrive - targetLong) / driveInv) * FRICTION_RELAX,
    -motor - contact.accLong,
    motor - contact.accLong
  );

  const latGrip = (reversing && wheel.steered ? 0.3 : 1) * contact.gripIn;
  const latRoom = maxFric * Math.max(0.15, rollScale) * latGrip;
  const wantLat = clampImpulse(
    (-slip.vLat / contact.invLat) * FRICTION_RELAX,
    -latRoom - contact.accLat,
    latRoom - contact.accLat
  );

  let nextLong = contact.accLong + wantLong;
  let nextLat = clampImpulse(contact.accLat + wantLat, -latRoom, latRoom);
  const mag = Math.hypot(nextLong, nextLat);
  if (mag > maxFric && mag > 1e-8) {
    const scale = maxFric / mag;
    nextLong *= scale;
    nextLat *= scale;
  }
  nextLong = clampImpulse(nextLong, -motor, motor);

  const driveX = driveStraight ? contact.cx : contact.tx;
  const driveY = driveStraight ? contact.cy : contact.ty;
  const driveZ = driveStraight ? contact.cz : contact.tz;
  applyAxisImpulse(chassis, contact, driveX, driveY, driveZ, nextLong - contact.accLong);
  contact.accLong = nextLong;
  applyLateral(chassis, contact, contact.sx, contact.sy, contact.sz, nextLat - contact.accLat);
  contact.accLat = nextLat;
}

function applyAxleYawDamping(
  chassis: RAPIER.RigidBody,
  axles: Map<string, KitAxleRuntime>,
  dt: number
): void {
  if (dt <= 0) return;
  const cr = chassis.rotation();
  scratchQuat.set(cr.x, cr.y, cr.z, cr.w);
  basisUp.set(0, 1, 0).applyQuaternion(scratchQuat);
  scratchForward.set(1, 0, 0).applyQuaternion(scratchQuat);
  for (const axle of axles.values()) {
    const ar = axle.body.rotation();
    const aq = new THREE.Quaternion(ar.x, ar.y, ar.z, ar.w);
    basis.set(1, 0, 0).applyQuaternion(aq);
    const alongUp = basis.dot(basisUp);
    basis.addScaledVector(basisUp, -alongUp);
    if (basis.lengthSq() < 1e-8) continue;
    basis.normalize();
    const lat = scratchLat.copy(scratchForward);
    lat.addScaledVector(basisUp, -lat.dot(basisUp));
    if (lat.lengthSq() < 1e-8) continue;
    lat.normalize();
    const cos = clampImpulse(basis.dot(lat), -1, 1);
    scratchRight.crossVectors(lat, basis);
    const sin = basisUp.dot(scratchRight);
    const yaw = Math.atan2(sin, cos);
    const abs = Math.abs(yaw);
    if (abs < AXLE_YAW_SOFT) continue;
    const excess = abs - AXLE_YAW_SOFT;
    const ang = axle.body.angvel();
    const omega = ang.x * basisUp.x + ang.y * basisUp.y + ang.z * basisUp.z;
    const sign = yaw >= 0 ? 1 : -1;
    let torque = -sign * AXLE_YAW_K * excess - AXLE_YAW_C * omega;
    torque = clampImpulse(torque, -AXLE_YAW_TORQUE_CAP, AXLE_YAW_TORQUE_CAP);
    const j = torque * dt;
    axle.body.applyTorqueImpulse(
      { x: basisUp.x * j, y: basisUp.y * j, z: basisUp.z * j },
      true
    );
  }
}

function softPlanarFollow(
  chassis: RAPIER.RigidBody,
  axles: Map<string, KitAxleRuntime>,
  dt: number,
  upright: number,
  flatPlant: boolean,
  noseUp: number,
  commandSpeed: number,
  throttle: number
): void {
  // Disabled: even idle XZ blending left axles in a state that tumbles under
  // coils+drive (foldDiag full vs fullNoPlanar). Revolute arms + Coulomb locate.
  void chassis;
  void axles;
  void dt;
  void upright;
  void flatPlant;
  void noseUp;
  void commandSpeed;
  void throttle;
}

function applyAirborneComMotor(
  chassis: RAPIER.RigidBody,
  commandSpeed: number,
  driveScale: number,
  maxAccel: number,
  maxForce: number,
  maxSpeed: number,
  dt: number,
  chassisUpright: number,
  upx: number,
  upy: number,
  upz: number
): void {
  // Crest/air launches: mild planar nudge only when upright and slow.
  if (chassisUpright < 0.78) return;
  basis.set(0, 0, -1).applyQuaternion(q);
  const intoUp = basis.x * upx + basis.y * upy + basis.z * upz;
  let fx = basis.x - upx * intoUp;
  let fy = basis.y - upy * intoUp;
  let fz = basis.z - upz * intoUp;
  const fLen = Math.hypot(fx, fy, fz);
  if (fLen < 1e-5) return;
  fx /= fLen;
  fy /= fLen;
  fz /= fLen;
  const lin = chassis.linvel();
  const planar = Math.hypot(lin.x, lin.z);
  if (planar > maxSpeed * 0.75) return;
  if (lin.y > 1.2) return;
  const vLong = lin.x * fx + lin.y * fy + lin.z * fz;
  const err = commandSpeed - vLong;
  const mass = Math.max(0.5, chassis.mass());
  const motorCap = Math.min(
    maxAccel * mass * dt * Math.max(0.2, driveScale) * 0.22,
    maxForce * dt * 0.22
  );
  const j = clampImpulse(err * mass * 0.4 * dt, -motorCap, motorCap);
  chassis.applyImpulse({ x: fx * j, y: fy * j, z: fz * j }, true);
}

function clampChassisSpeed(
  chassis: RAPIER.RigidBody,
  maxSpeed: number,
  upright: number,
  hasFace: boolean,
  flatPlant: boolean,
  onIncline = false
): void {
  const vel = chassis.linvel();
  const planar = Math.hypot(vel.x, vel.z);
  const speedCap =
    (flatPlant ? maxSpeed * 1.1 : maxSpeed * 0.8) *
    (upright < 0.75 ? 0.5 : upright < 0.85 ? 0.75 : 1);
  let vx = vel.x;
  let vy = vel.y;
  let vz = vel.z;
  if (planar > speedCap && planar > 1e-6) {
    const scale = speedCap / planar;
    vx *= scale;
    vz *= scale;
  }
  // Incline: slightly tighter loft to settle front hop without starving crest.
  const launchCap = onIncline ? 1.05 : hasFace ? 2.0 : flatPlant ? 1.1 : 1.4;
  if (vy > launchCap) vy = launchCap;
  if (onIncline && vy > 0.55) vy = 0.55 + (vy - 0.55) * 0.5;
  const total = Math.hypot(vx, vy, vz);
  const totalCap = Math.max(speedCap * 1.25, launchCap + speedCap * 0.85);
  if (total > totalCap && total > 1e-6) {
    const s = totalCap / total;
    vx *= s;
    vy *= s;
    vz *= s;
  }
  if (vx !== vel.x || vy !== vel.y || vz !== vel.z) {
    chassis.setLinvel({ x: vx, y: vy, z: vz }, true);
  }
}

export function applyHubDrive(
  world: RAPIER.World,
  chassis: RAPIER.RigidBody,
  axles: Map<string, KitAxleRuntime>,
  wheels: { def: KitWheelDef; axle: KitAxleRuntime }[],
  rig: RigDef,
  input: DriveInput,
  state: HubDriveState,
  dt: number
): void {
  if (dt <= 0 || wheels.length === 0) return;

  const drive = rig.kit?.drive ?? {
    mu: rig.suspension.mu,
    driveTorque: rig.suspension.driveTorque,
    maxForce: rig.suspension.maxForce,
    steerAngle: rig.suspension.steerAngle,
    maxAccel: rig.suspension.maxAccel,
    minNormalY: rig.suspension.minNormalY,
    minUpright: rig.suspension.minUpright,
  };

  const desired = input.throttle * rig.maxSpeed;
  const step = drive.maxAccel * dt;
  state.commandSpeed += clampImpulse(desired - state.commandSpeed, -step, step);
  const blend = 1 - Math.exp(-dt / 0.08);
  state.steer += (input.steer - state.steer) * blend;
  if (input.steer === 0 && Math.abs(state.steer) < 0.02) state.steer = 0;
  const steerCmd = state.steer;

  const rot = chassis.rotation();
  q.set(rot.x, rot.y, rot.z, rot.w);
  basisUp.set(0, 1, 0).applyQuaternion(q);

  const gravity = world.gravity;
  const gx = gravity.x;
  const gy = gravity.y;
  const gz = gravity.z;
  const gMag = Math.hypot(gx, gy, gz) || 9.81;
  const upx = -gx / gMag;
  const upy = -gy / gMag;
  const upz = -gz / gMag;
  const upright = basisUp.x * upx + basisUp.y * upy + basisUp.z * upz;
  // Torque-only yaw always (no pose snaps). Soft planar deferred until contacts known.
  applyAxleYawDamping(chassis, axles, dt);
  // Tip recovery when past crawl pitch into tumble.
  if (upright < 0.7 && upright > 0.12) {
    const mass = Math.max(0.5, chassis.mass());
    basis.set(0, 0, -1).applyQuaternion(q);
    const noseUp = basis.x * upx + basis.y * upy + basis.z * upz;
    if (Math.abs(noseUp) > 0.04) {
      basis.set(1, 0, 0).applyQuaternion(q);
      const torque = -noseUp * 3.0 * mass * dt;
      chassis.applyTorqueImpulse(
        { x: basis.x * torque, y: basis.y * torque, z: basis.z * torque },
        true
      );
    }
  }
  // Bleed tumble spin only when truly tipping — earlier bleed killed steer yaw.
  if (upright < 0.55 && upright > 0.12) {
    const ang = chassis.angvel();
    const bleed = upright < 0.4 ? 0.5 : 0.72;
    chassis.setAngvel(
      { x: ang.x * bleed, y: ang.y * 0.95, z: ang.z * bleed },
      true
    );
  }
  if (upright < drive.minUpright) return;

  const hubWheels: HubWheel[] = wheels.map((w) => ({
    def: w.def,
    axle: w.axle,
    sideX: w.def.hubOffset.x,
    steered: w.def.steered,
    driven: w.def.driven,
    radius: w.def.radius,
    mu: drive.mu,
  }));

  const contacts: HubContact[] = [];
  const weight = chassis.mass() * gMag;
  const wantProbe =
    input.throttle !== 0 || Math.abs(state.commandSpeed) > 0.05;
  const impactSpeedNow = Math.hypot(
    chassis.linvel().x,
    chassis.linvel().y,
    chassis.linvel().z
  );
  // Arm sticky hold on ballistic speeds (axle-ram ~4.5). Trail ramp peaks ~3.3.
  if (impactSpeedNow > 3.4) state.impactHold = Math.max(state.impactHold, 2.0);
  state.impactHold = Math.max(0, state.impactHold - dt);
  // Face probes at crawl approach; suppress casts during ballistic hold (axle-ram).
  const wantFaceProbe =
    wantProbe && impactSpeedNow <= 2.5 && state.impactHold <= 0;

  // Cast lips in the travel direction so reverse climbs find aft risers.
  const driveSign =
    Math.abs(state.commandSpeed) > 0.05
      ? Math.sign(state.commandSpeed)
      : input.throttle !== 0
        ? Math.sign(input.throttle)
        : 1;
  basis.set(0, 0, -1).applyQuaternion(q);
  // Forward: chassis nose. Reverse: cast aft so stair/ledge risers behind the hubs.
  const faceFx = basis.x * driveSign;
  const faceFy = basis.y * driveSign;
  const faceFz = basis.z * driveSign;
  const faceNoseUp = faceFx * upx + faceFy * upy + faceFz * upz;
  // Skid / nose-up: bias lip casts more upward so hanging hubs still find the riser.
  const lipUpBias = 0.4 + Math.max(0, faceNoseUp) * 0.6;

  const pushHit = (
    wheel: HubWheel,
    hit: { toi: number; nx: number; ny: number; nz: number; support: number },
    faceBoost: boolean
  ): void => {
    const hub = hubWorldPosition(wheel.axle, wheel.def.hubOffset);
    const penet = Math.max(0, wheel.radius - hit.toi);
    const share = weight / Math.max(1, hubWheels.length);
    // Steep/face contacts get little weight·support — penetration floor keeps μFn alive.
    const supportFn = share * 1.25 * Math.max(hit.support, faceBoost ? 0.85 : 0);
    const fn = Math.min(supportFn + penet * 340, share * (faceBoost ? 3.6 : 1.85));
    if (fn <= 1e-3) return;

    const nx = hit.nx;
    const ny = hit.ny;
    const nz = hit.nz;
    const px = hub.x - nx * wheel.radius;
    const py = hub.y - ny * wheel.radius;
    const pz = hub.z - nz * wheel.radius;

    const steer = wheel.steered ? steerCmd * drive.steerAngle : 0;
    basis.set(0, 0, -1).applyQuaternion(q);
    const chassisInto = basis.x * nx + basis.y * ny + basis.z * nz;
    let cx = basis.x - nx * chassisInto;
    let cy = basis.y - ny * chassisInto;
    let cz = basis.z - nz * chassisInto;
    const cLen = Math.hypot(cx, cy, cz);
    if (cLen < 1e-5) return;
    cx /= cLen;
    cy /= cLen;
    cz /= cLen;

    if (steer !== 0) basis.applyAxisAngle(basisUp, steer);
    let tx: number;
    let ty: number;
    let tz: number;
    if (faceBoost || ny < 0.5) {
      // Travel-aligned preferred dir (aft when reversing) keeps world-up claw.
      const climb = faceClimbTangent(
        nx,
        ny,
        nz,
        basis.x * driveSign,
        basis.y * driveSign,
        basis.z * driveSign,
        upx,
        upy,
        upz
      );
      if (!climb) return;
      tx = climb.tx;
      ty = climb.ty;
      tz = climb.tz;
    } else {
      const into = basis.x * nx + basis.y * ny + basis.z * nz;
      tx = basis.x - nx * into;
      ty = basis.y - ny * into;
      tz = basis.z - nz * into;
      const tLen = Math.hypot(tx, ty, tz);
      if (tLen < 1e-5) return;
      tx /= tLen;
      ty /= tLen;
      tz /= tLen;
    }

    let sx = ny * tz - nz * ty;
    let sy = nz * tx - nx * tz;
    let sz = nx * ty - ny * tx;
    const sLen = Math.hypot(sx, sy, sz);
    if (sLen < 1e-5) return;
    sx /= sLen;
    sy /= sLen;
    sz /= sLen;

    const gripIn = 0.55 + 0.45 * clampImpulse(1 - penet / Math.max(1e-3, wheel.radius * 0.15), 0, 1);
    const gripSupport = faceBoost
      ? Math.max(hit.support, drive.minNormalY + 0.18)
      : hit.support;
    contacts.push({
      wheel,
      fn,
      px,
      py,
      pz,
      nx,
      ny,
      nz,
      tx,
      ty,
      tz,
      cx,
      cy,
      cz,
      sx,
      sy,
      sz,
      invLong: invMassAlong(chassis, px, py, pz, tx, ty, tz),
      invChassis: invMassAlong(chassis, px, py, pz, cx, cy, cz),
      invLat: invMassAlong(chassis, px, py, pz, sx, sy, sz),
      accLong: 0,
      accLat: 0,
      grip: Math.max(faceBoost ? 0.7 : 0.35, gripFromSupport(gripSupport, drive.minNormalY)),
      gripIn,
      // Face-cast lips only — not every shallow incline (that tipped stair noses).
      face: faceBoost,
    });
  };

  // Axle height split: fronts already on a stair/ledge deck while rears are still
  // low — force face grabs on the low axle even when it has flat down-support.
  let frontHubY = 0;
  let rearHubY = 0;
  let nFrontHub = 0;
  let nRearHub = 0;
  for (const wheel of hubWheels) {
    const hub = hubWorldPosition(wheel.axle, wheel.def.hubOffset);
    if (wheel.steered) {
      frontHubY += hub.y;
      nFrontHub += 1;
    } else {
      rearHubY += hub.y;
      nRearHub += 1;
    }
  }
  const frontHubYAvg = nFrontHub > 0 ? frontHubY / nFrontHub : 0;
  const rearHubYAvg = nRearHub > 0 ? rearHubY / nRearHub : 0;
  const axleSplitY = frontHubYAvg - rearHubYAvg;
  // ~0.9R — ignore mild articulation; real stair/ledge hang is taller.
  const straddleClimb = Math.abs(axleSplitY) > 0.055;
  const lowAxleIsRear = axleSplitY > 0.055;
  const lowAxleIsFront = axleSplitY < -0.055;

  for (const wheel of hubWheels) {
    const hub = hubWorldPosition(wheel.axle, wheel.def.hubOffset);
    const supportHit = castHubSupport(
      world,
      hub,
      wheel.radius,
      wheel.axle.body,
      gx,
      gy,
      gz,
      drive.minNormalY
    );
    if (supportHit) {
      // Only true verticals get face treatment — 45° noses stay normal plant.
      // Skip face-tag at high speed so ledge rams don't claw mid-impact.
      pushHit(wheel, supportHit, wantFaceProbe && supportHit.ny < 0.4);
    } else if (wantFaceProbe) {
      // No down-plant: still need face/lip so hanging hubs (skid high-center) claw.
      const faceHit = castHubFace(
        world,
        hub,
        wheel.radius,
        wheel.axle.body,
        faceFx,
        faceFy,
        faceFz,
        gx,
        gy,
        gz
      );
      if (faceHit) pushHit(wheel, faceHit, true);
      else {
        const lipHit = castHubFace(
          world,
          hub,
          wheel.radius,
          wheel.axle.body,
          faceFx + upx * lipUpBias,
          faceFy + upy * lipUpBias,
          faceFz + upz * lipUpBias,
          gx,
          gy,
          gz
        );
        if (lipHit) pushHit(wheel, lipHit, true);
      }
    }

    // (Planted steep-lip probe omitted — WOT into ramp edge tips.)

    // Face/lip for driven hubs that lack a solid down-plant (skid hang / steep
    // reject). Tall ledge hang: low axle while the other sits on the deck.
    const tallLedgeHang =
      impactSpeedNow <= 1.4 &&
      ((lowAxleIsRear && !wheel.steered && axleSplitY > 0.12) ||
        (driveSign < 0 && lowAxleIsFront && wheel.steered && axleSplitY < -0.12));
    const needsFaceGrab =
      wantFaceProbe &&
      wheel.driven &&
      (!supportHit ||
        supportHit.ny < 0.5 ||
        supportHit.support < Math.max(0.28, drive.minNormalY) ||
        tallLedgeHang);
    if (needsFaceGrab) {
      const reachMul = tallLedgeHang ? 2.2 : 1;
      const alreadySteepFace = contacts.some(
        (c) => c.wheel === wheel && c.face && c.ny < 0.7
      );
      if (!alreadySteepFace) {
        const faceHit = castHubFace(
          world,
          hub,
          wheel.radius,
          wheel.axle.body,
          faceFx,
          faceFy,
          faceFz,
          gx,
          gy,
          gz,
          reachMul
        );
        if (faceHit && faceHit.support < 0.8 && (!tallLedgeHang || faceHit.ny < 0.45)) {
          pushHit(wheel, faceHit, true);
        } else {
          const lipHit = castHubFace(
            world,
            hub,
            wheel.radius,
            wheel.axle.body,
            faceFx + upx * lipUpBias,
            faceFy + upy * lipUpBias,
            faceFz + upz * lipUpBias,
            gx,
            gy,
            gz,
            reachMul
          );
          if (lipHit && lipHit.support < 0.8 && (!tallLedgeHang || lipHit.ny < 0.45)) {
            pushHit(wheel, lipHit, true);
          }
        }
      }
      // Rear skid rescue (and reverse front hang): down-travel cast finds the riser.
      if (!wheel.steered || (driveSign < 0 && wheel.steered && tallLedgeHang)) {
        const stillNoFace = !contacts.some((c) => c.wheel === wheel && c.face);
        if (stillNoFace) {
          const underHit = castHubFace(
            world,
            hub,
            wheel.radius * 1.25,
            wheel.axle.body,
            faceFx - upx * 0.3,
            faceFy - upy * 0.3,
            faceFz - upz * 0.3,
            gx,
            gy,
            gz,
            reachMul
          );
          if (underHit && underHit.support < 0.85 && (!tallLedgeHang || underHit.ny < 0.45)) {
            pushHit(wheel, underHit, true);
          }
        }
      }
    }
  }

  if (contacts.length === 0) {
    // Airborne: planar COM assist only (no Coulomb without plant).
    if (input.throttle !== 0 || Math.abs(state.commandSpeed) > 0.05) {
      applyAirborneComMotor(
        chassis,
        state.commandSpeed,
        clampImpulse((upright - 0.78) / 0.18, 0, 1),
        drive.maxAccel,
        drive.maxForce,
        rig.maxSpeed,
        dt,
        upright,
        upx,
        upy,
        upz
      );
    }
    clampChassisSpeed(chassis, rig.maxSpeed, upright, false, false);
    return;
  }

  let flatSupport = 0;
  let supportCount = 0;
  for (const c of contacts) {
    // Face/lip plants must not spoil flatPlant (brief hop face casts).
    if (c.face) continue;
    flatSupport += c.ny;
    supportCount += 1;
  }
  basis.set(0, 0, -1).applyQuaternion(q);
  const noseForPlanar = basis.x * upx + basis.y * upy + basis.z * upz;
  // Gentle trail ramps (~16°, ny≈0.96) stay out via ny threshold alone.
  const flatPlant =
    supportCount >= 2 &&
    flatSupport / supportCount > 0.97 &&
    upright > 0.92;
  if (!kitDiagFlags.skipSoftPlanar) {
    softPlanarFollow(
      chassis,
      axles,
      dt,
      upright,
      flatPlant,
      noseForPlanar,
      state.commandSpeed,
      input.throttle
    );
  }

  let frontFn = 0;
  let rearFn = 0;
  let drivenContacts = 0;
  for (const contact of contacts) {
    if (contact.wheel.driven) drivenContacts += 1;
    if (contact.wheel.steered) frontFn += contact.fn;
    else rearFn += contact.fn;
  }

  let zFront = 0;
  let zRear = 0;
  let nF = 0;
  let nR = 0;
  for (const wheel of hubWheels) {
    const z = wheel.axle.restLocal.z;
    if (wheel.steered) {
      zFront += z;
      nF += 1;
    } else {
      zRear += z;
      nR += 1;
    }
  }
  const wheelbase =
    nF > 0 && nR > 0 ? Math.max(0.2, Math.abs(zFront / nF - zRear / nR)) : 0.313;

  const planted = 0.32 * weight;
  // Face plants count for scale even when the loaded axle is airborne (skid / hang).
  let faceFn = 0;
  let rearFaceFn = 0;
  let frontFaceFn = 0;
  for (const c of contacts) {
    if (!c.face || !c.wheel.driven) continue;
    faceFn += c.fn;
    if (!c.wheel.steered) rearFaceFn += c.fn;
    else frontFaceFn += c.fn;
  }
  // Inclines/faces often unload the "drive" axle — floor so Coulomb still pulls.
  const hasFace = contacts.some((c) => c.face || c.ny < 0.85);
  const rearClimbing = rearFaceFn > 1e-3 && faceFn > frontFn * 0.85;
  // Reverse stair/ledge: fronts claw the riser while rears sit on the deck.
  const frontClimbing =
    driveSign < 0 && frontFaceFn > 1e-3 && faceFn > rearFn * 0.85;
  // Incline plant (trail ramp) — not a vertical face, but not flat either.
  const onIncline =
    !flatPlant &&
    supportCount >= 2 &&
    flatSupport / supportCount < 0.985 &&
    flatSupport / supportCount > 0.55;
  // Flat-tread axle split (stairs / ledge hang) — not trail-ramp articulation.
  const stairHang = straddleClimb && !onIncline;
  // Hang: keep scale from the loaded axle (primary axle alone starves skid climb).
  const rawDriveScale = clampImpulse(
    (rearClimbing || frontClimbing || stairHang
      ? Math.max(frontFn, rearFn, faceFn)
      : state.commandSpeed >= 0
        ? Math.max(frontFn, faceFn)
        : Math.max(rearFn, faceFn)) / planted,
    0,
    1
  );
  // Planted on an obstacle top (ledge deck) — kill leftover claw/thrust that tips.
  // Require near-world-up normals so gentle trail ramps (ny≈0.96) are not "on deck".
  // Axle height split means the low axle is still climbing — never "on deck".
  const onDeck =
    !stairHang &&
    !rearClimbing &&
    !frontClimbing &&
    contacts.length >= 2 &&
    contacts.filter((c) => c.ny > 0.97).length >= 2 &&
    !onIncline &&
    !contacts.some((c) => c.face && c.ny < 0.5) &&
    chassis.translation().y > 0.2;
  const driveScale = !flatPlant || hasFace ? Math.max(0.55, rawDriveScale) : Math.max(0.7, rawDriveScale);
  const rollScale = rollAuthority(contacts, weight);
  // Slopes/faces get full scale; flat stays 1 (fold controlled via perWheel multiplier).
  const slopeScale = 1;
  const attitudeScale = clampImpulse((upright - 0.55) / 0.35, 0.2, 1);
  basis.set(0, 0, -1).applyQuaternion(q);
  const noseUpEarly = basis.x * upx + basis.y * upy + basis.z * upz;
  // Crest risk: tipping, or nose-high onto a near-level deck — not lip approach
  // with rears still on flat behind a steep face / axle straddle.
  const crestRisk =
    upright < 0.62 ||
    (noseUpEarly > 0.16 &&
      !onIncline &&
      !stairHang &&
      !contacts.some((c) => c.face && c.ny < 0.5) &&
      contacts.filter((c) => !c.face && c.ny > 0.97).length >= 2);
  // Kill thrust early into a tip — crest launches otherwise invert.
  const tipCut = upright < 0.68 ? 0 : upright < 0.78 ? 0.3 : upright < 0.88 ? 0.65 : 1;
  // Wheelie cut: only real wheelies on flat — mild nose-up is normal under WOT.
  const pitchCut = flatPlant
    ? noseUpEarly > 0.28
      ? clampImpulse(1 - (noseUpEarly - 0.28) / 0.35, 0.45, 1)
      : 1
    : crestRisk && noseUpEarly > 0.36
      ? clampImpulse(1 - (noseUpEarly - 0.36) / 0.28, 0.2, 1)
      : 1;
  // Reverse hang pitches nose down — don't starve climb as a "dive".
  const diveCut =
    driveSign < 0
      ? 1
      : noseUpEarly < -0.15
        ? clampImpulse(1 + (noseUpEarly + 0.15) / 0.3, 0.2, 1)
        : 1;
  // Hang / skid face: keep climb mul high so the low axle can claw.
  const climbMul = flatPlant
    ? 1.9
    : rearClimbing || frontClimbing
      ? 3.05
      : hasFace
        ? 2.7
        : onIncline
          ? 2.55
          : 2.5;
  // Crest: nose-high onto a deck — cut thrust before invert (not mid-ramp / stair hang).
  const crestCut =
    !flatPlant && crestRisk && noseUpEarly > 0.26
      ? clampImpulse(1 - (noseUpEarly - 0.26) / 0.3, 0.15, 1)
      : 1;
  const deckCut = onDeck ? (noseUpEarly > 0.08 ? 0.15 : 0.45) : 1;
  const impactSpeed = Math.hypot(
    chassis.linvel().x,
    chassis.linvel().y,
    chassis.linvel().z
  );
  const steepFaceHit = contacts.some((c) => c.face && c.ny < 0.45);
  // Soft hold only for real flat→wall rams. Crawl onto ramp/stair lips (~≤2.5)
  // must keep drive — low threshold starved headway / felt like bounce-stuck.
  if (steepFaceHit && flatPlant && impactSpeed > 2.8) {
    state.impactHold = Math.max(
      state.impactHold,
      clampImpulse(0.35 + (impactSpeed - 2.8) * 0.4, 0.35, 1.4)
    );
  }
  if (!onIncline && !stairHang && impactSpeed > 3.4) {
    state.impactHold = Math.max(state.impactHold, 2.0);
  }
  // Continuous damp only on faster flat→face rams (not crawl lip cresting).
  const faceSpeedCut =
    steepFaceHit && flatPlant && impactSpeed > 2.2
      ? clampImpulse(1 - (impactSpeed - 2.2) / 1.6, 0.2, 1)
      : 1;
  // Incline/stair: full drive. Hold: hard cut only while still ballistic.
  const impactCut =
    onIncline || stairHang
      ? 1
      : state.impactHold > 0
        ? impactSpeed > 3.0
          ? 0.02
          : 0.55
        : faceSpeedCut;
  const perWheelCap =
    (drive.maxAccel *
      Math.max(0.5, chassis.mass()) *
      dt *
      Math.max(0.55, driveScale) *
      // Flat: enough to reach trails. Faces/ledges: hard pull.
      climbMul *
      attitudeScale *
      pitchCut *
      diveCut *
      slopeScale *
      tipCut *
      crestCut *
      deckCut *
      impactCut) /
    Math.max(1, drivenContacts);

  basis.set(0, 0, -1).applyQuaternion(q);
  const lin = chassis.linvel();
  const fwdSpeed = lin.x * basis.x + lin.y * basis.y + lin.z * basis.z;
  const travel =
    Math.abs(fwdSpeed) > 0.25 ? Math.sign(fwdSpeed) : Math.sign(state.commandSpeed) || 1;
  const reversing = travel < 0;

  const wantDrive =
    input.throttle !== 0 ||
    input.steer !== 0 ||
    Math.abs(state.commandSpeed) > 0.05 ||
    Math.abs(steerCmd) > 0.02;
  // Idle: light lateral Coulomb keeps hub plant from skating (hub Rapier μ = 0).
  if (!wantDrive) {
    for (let iteration = 0; iteration < 3; iteration += 1) {
      for (const contact of contacts) {
        const slip = tangentVelocity(chassis, contact, gx, gy, gz, dt);
        const maxFric = frictionLimit(contact, drive.maxForce) * dt * 0.35;
        const wantLat = clampImpulse(
          (-slip.vLat / contact.invLat) * 0.5,
          -maxFric - contact.accLat,
          maxFric - contact.accLat
        );
        applyLateral(chassis, contact, contact.sx, contact.sy, contact.sz, wantLat);
        contact.accLat += wantLat;
      }
    }
    return;
  }

  // Planted: Coulomb owns long/lat. Face/lip: tire pull-up at the contact +
  // COM climb assist along up-biased face tangents.
  {
    const mass = Math.max(0.5, chassis.mass());

    // Tire claw: each steep face contact pulls along its climb tangent at the
    // tread — the "grab the lip and haul up" feel (COM shove alone feels like a nudge).
    const hasSteepFace = contacts.some((c) => c.face && c.ny < 0.45);
    // Real crest onto a near-level deck — not lip approach with rears still on flat
    // behind a steep face (that false-crest starved ramp_from_flat / ledge crawl).
    const cresting =
      !stairHang &&
      !rearClimbing &&
      !frontClimbing &&
      noseUpEarly > 0.18 &&
      !contacts.some((c) => c.face && c.ny < 0.5) &&
      contacts.filter((c) => !c.face && c.ny > 0.97).length >= 2;
    if (
      !cresting &&
      // Lip crawl: allow mild nose-up into the face (strict gate stuck the approach).
      noseUpEarly < (onIncline || stairHang || steepFaceHit ? 0.42 : 0.32) &&
      Math.abs(state.commandSpeed) > 0.05 &&
      upright > 0.62 &&
      tipCut > 0.2 &&
      impactCut > 0.15
    ) {
      for (const c of contacts) {
        const nyCut = c.wheel.steered ? 0.55 : 0.65;
        if (!c.face || !c.wheel.driven || c.ny > nyCut) continue;
        // Mid-ramp: fronts on the deck must not claw (front hop). Steep lips OK.
        if (c.wheel.steered && onIncline && !frontClimbing && c.ny > 0.4) continue;
        // Front claw off during high-speed face impacts (ram tip-over).
        if (c.wheel.steered && impactCut < 0.5) continue;
        // Scale claw while closing into the face — hard skip starved crawl headway.
        // Dead-stick lip (slow) keeps full claw so settle-on-lip can still crest.
        const closing = -(lin.x * c.nx + lin.y * c.ny + lin.z * c.nz);
        const closeScale =
          impactSpeed < 0.85
            ? 1
            : clampImpulse(1 - (closing - 0.5) / 0.9, 0, 1);
        if (closeScale < 0.05) continue;
        const steep = clampImpulse(1 - c.ny, 0.25, 1);
        const vClimb = lin.x * c.tx + lin.y * c.ty + lin.z * c.tz;
        // Reverse face tx is travel-aligned — chase abs climb speed.
        const want =
          (driveSign < 0 ? Math.abs(state.commandSpeed) : state.commandSpeed) *
          (0.55 + 0.45 * steep);
        const err = want - vClimb;
        const axlePull = c.wheel.steered ? (frontClimbing ? 0.4 : 0.28) : 0.44;
        const pullScale =
          steep *
          tipCut *
          crestCut *
          impactCut *
          closeScale *
          clampImpulse((upright - 0.62) / 0.28, 0.25, 1);
        const pullCap = Math.min(
          drive.maxAccel * mass * dt * axlePull * pullScale,
          frictionLimit(c, drive.maxForce) * dt * 0.75 * pullScale,
          drive.maxForce * dt * 0.55 * pullScale
        );
        const jp = clampImpulse(err * mass * 0.75 * dt, -pullCap, pullCap);
        chassis.applyImpulseAtPoint(
          { x: c.tx * jp, y: c.ty * jp, z: c.tz * jp },
          { x: c.px, y: c.py, z: c.pz },
          true
        );
      }
    }

    let clx = 0,
      cly = 0,
      clz = 0,
      nClimb = 0;
    for (const c of contacts) {
      if (
        c.face ||
        c.ny < 0.88 ||
        (onIncline && c.ny < 0.985) ||
        (!flatPlant &&
          (rearClimbing || frontClimbing || stairHang) &&
          c.ny < 0.985)
      ) {
        clx += c.tx;
        cly += c.ty;
        clz += c.tz;
        nClimb += 1;
      }
    }
    if (
      nClimb > 0 &&
      Math.abs(state.commandSpeed) > 0.05 &&
      upright > 0.65 &&
      tipCut > 0.2 &&
      impactCut > 0.15 &&
      !cresting &&
      (!onDeck || stairHang || rearClimbing || frontClimbing) &&
      // Lip / straddle crawl needs haul even with mild nose-up into the face.
      noseUpEarly < (stairHang || steepFaceHit ? 0.42 : 0.28) &&
      // No COM haul while ballistic-slamming a steep face on flat (bounce/tip).
      !(steepFaceHit && flatPlant && !stairHang && impactSpeed > 2.6)
    ) {
      clx /= nClimb;
      cly /= nClimb;
      clz /= nClimb;
      const cLen = Math.hypot(clx, cly, clz);
      if (cLen > 1e-5) {
        clx /= cLen;
        cly /= cLen;
        clz /= cLen;
        const upAlong = clx * upx + cly * upy + clz * upz;
        if (upAlong < 0.2 && hasSteepFace) {
          clx += upx * 0.28;
          cly += upy * 0.28;
          clz += upz * 0.28;
          const n2 = Math.hypot(clx, cly, clz);
          if (n2 > 1e-5) {
            clx /= n2;
            cly /= n2;
            clz /= n2;
          }
        }
        const vClimb = lin.x * clx + lin.y * cly + lin.z * clz;
        const climbTarget =
          driveSign < 0 && hasFace ? Math.abs(state.commandSpeed) : state.commandSpeed;
        const climbErr = climbTarget - vClimb;
        const climbScale =
          clampImpulse((upright - 0.65) / 0.28, 0.25, 1) * tipCut * crestCut * impactCut;
        const faceMul = hasFace ? 1.65 : 1.15;
        const climbCap = Math.min(
          drive.maxAccel * mass * dt * Math.max(0.65, driveScale) * faceMul * climbScale,
          drive.maxForce * dt * 1.05 * climbScale
        );
        const jc = clampImpulse(climbErr * mass * 0.9 * dt, -climbCap, climbCap);
        chassis.applyImpulse({ x: clx * jc, y: cly * jc, z: clz * jc }, true);
      }
    }

    // Stuck / crawl lip: when nearly stopped against a face or axle-straddle
    // hang, haul COM uphill. Under-settled idle on the ramp lip otherwise
    // dead-sticks (bounce, no headway) because flatPlant zeroes nClimb.
    const lipStuck =
      !onIncline &&
      impactSpeed < 0.7 &&
      Math.abs(state.commandSpeed) > 0.25 &&
      upright > 0.82 &&
      tipCut > 0.45 &&
      Math.abs(lin.x * basis.x + lin.y * basis.y + lin.z * basis.z) < 0.35 &&
      (steepFaceHit || stairHang);
    if (lipStuck) {
      let lx = 0,
        ly = 0,
        lz = 0,
        n = 0;
      for (const c of contacts) {
        // Include mild ramp plants during straddle — nClimb skips them when flatPlant.
        if (!(c.face || c.ny < 0.985)) continue;
        lx += c.tx;
        ly += c.ty;
        lz += c.tz;
        n += 1;
      }
      if (n === 0) {
        // No useful contacts: still shove chassis forward+up out of the skid hang.
        basis.set(0, 0, -1).applyQuaternion(q);
        lx = basis.x * driveSign + upx * 0.45;
        ly = basis.y * driveSign + upy * 0.45;
        lz = basis.z * driveSign + upz * 0.45;
        n = 1;
      } else {
        lx /= n;
        ly /= n;
        lz /= n;
        lx += upx * 0.28;
        ly += upy * 0.28;
        lz += upz * 0.28;
      }
      const len = Math.hypot(lx, ly, lz);
      if (len > 1e-5) {
        lx /= len;
        ly /= len;
        lz /= len;
        const vClimb = lin.x * lx + lin.y * ly + lin.z * lz;
        const want = Math.abs(state.commandSpeed) * 0.9;
        const err = want - vClimb;
        const cap = Math.min(
          drive.maxAccel * mass * dt * 1.2,
          drive.maxForce * dt * 0.75
        );
        const j = clampImpulse(err * mass * 0.85 * dt, -cap, cap);
        chassis.applyImpulse({ x: lx * j, y: ly * j, z: lz * j }, true);
      }
    }

    // Mild drive-only pitch restore (chassis attitude). Sign follows nose.
    if (upright < 0.95 && upright > 0.4) {
      basis.set(0, 0, -1).applyQuaternion(q);
      const noseUp = basis.x * upx + basis.y * upy + basis.z * upz;
      if (Math.abs(noseUp) > 0.05) {
        basis.set(1, 0, 0).applyQuaternion(q);
        // Flat/deck: kill wheelies. Crest: stronger. Mid-face: light so tire
        // pull can pitch the nose onto the lip.
        const gain = flatPlant || onDeck ? 1.1 : noseUp > 0.35 ? 1.6 : hasFace ? 0.25 : 0.4;
        const torque = -noseUp * gain * mass * dt;
        chassis.applyTorqueImpulse(
          { x: basis.x * torque, y: basis.y * torque, z: basis.z * torque },
          true
        );
      }
    }
  }

  for (let iteration = 0; iteration < FRICTION_ITERATIONS; iteration += 1) {
    for (const contact of contacts) {
      solveContact(
        chassis,
        contact,
        drive.maxForce,
        state.commandSpeed,
        steerCmd,
        perWheelCap,
        rollScale,
        reversing,
        gx,
        gy,
        gz,
        dt
      );
    }
  }

  if (reversing) {
    applyReverseYaw(
      chassis,
      drive.steerAngle,
      steerCmd,
      fwdSpeed,
      wheelbase,
      gx,
      gy,
      gz,
      gMag,
      rollScale
    );
  } else {
    applySteer(chassis, contacts, drive.maxForce, steerCmd, dt, rollScale);
  }

  clampChassisSpeed(chassis, rig.maxSpeed, upright, hasFace, flatPlant, onIncline);
}
