/**
 * Phase 4 hub Coulomb grip: contact + drive at axle hubs.
 * Soft hub spheres = only vertical plant. Coilovers = ride height. Links = locate.
 * No chassis-ray spring and no ray normal impulse (would double-plant / self-push).
 * Fn for μ·Fn is a weight/penetration estimate only. Reverse steer yaws the tail.\n * Longitudinal Coulomb along contact tangent (includes uphill) restores climb;
 * planar COM motor is a flat-drive assist. Forward face probe aids vertical lips.
 */
import RAPIER from "@dimforge/rapier3d-compat";
import * as THREE from "three";
import type { DriveInput } from "../input/Input.ts";
import type { KitWheelDef, RigDef } from "./types.ts";
import type { KitAxleRuntime } from "./kitBodies.ts";
import { hubWorldPosition } from "./kitBodies.ts";

const FRICTION_ITERATIONS = 6;
const FRICTION_RELAX = 0.35;
const ROLL_KEEP = 0.18;

export type HubDriveState = {
  commandSpeed: number;
  steer: number;
};

export function createHubDriveState(): HubDriveState {
  return { commandSpeed: 0, steer: 0 };
}

export function resetHubDriveState(state: HubDriveState): void {
  state.commandSpeed = 0;
  state.steer = 0;
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
};

const q = new THREE.Quaternion();
const basis = new THREE.Vector3();
const basisUp = new THREE.Vector3();
const scratchForward = new THREE.Vector3();
const scratchQuat = new THREE.Quaternion();

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
  gz: number
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
  const hit = world.castRayAndGetNormal(ray, radius * 1.35, false, undefined, undefined, undefined, axleBody);
  if (!hit) return null;
  if (hit.timeOfImpact > radius * 1.15) return null;
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
  chassis.applyImpulseAtPoint(
    { x: ax * impulse, y: ay * impulse, z: az * impulse },
    { x: contact.px, y: contact.py, z: contact.pz },
    true
  );
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
    const jLat = clampImpulse(steerInput * cap * 0.1, -room, room);
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
  const driveStraight = reversing && wheel.steered;
  const vDrive = driveStraight ? slip.vChassis : slip.vLong;
  const targetLong = wheel.driven ? commandSpeed : vDrive;
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

function syncAxlePlanar(chassis: RAPIER.RigidBody, axles: Map<string, KitAxleRuntime>): void {
  const cl = chassis.linvel();
  for (const axle of axles.values()) {
    const al = axle.body.linvel();
    axle.body.setLinvel({ x: cl.x, y: al.y, z: cl.z }, true);
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

  basis.set(0, 0, -1).applyQuaternion(q);
  const faceFx = basis.x;
  const faceFy = basis.y;
  const faceFz = basis.z;

  const pushHit = (
    wheel: HubWheel,
    hit: { toi: number; nx: number; ny: number; nz: number; support: number },
    faceBoost: boolean
  ): void => {
    const hub = hubWorldPosition(wheel.axle, wheel.def.hubOffset);
    const penet = Math.max(0, wheel.radius - hit.toi);
    const share = weight / Math.max(1, hubWheels.length);
    // Steep/face contacts get little weight·support — penetration floor keeps μFn alive.
    const supportFn = share * 1.1 * Math.max(hit.support, faceBoost ? 0.35 : 0);
    const fn = Math.min(supportFn + penet * 220, share * (faceBoost ? 2.2 : 1.6));
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
    const into = basis.x * nx + basis.y * ny + basis.z * nz;
    let tx = basis.x - nx * into;
    let ty = basis.y - ny * into;
    let tz = basis.z - nz * into;
    const tLen = Math.hypot(tx, ty, tz);
    if (tLen < 1e-5) return;
    tx /= tLen;
    ty /= tLen;
    tz /= tLen;

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
      ? Math.max(hit.support, drive.minNormalY + 0.05)
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
      grip: Math.max(0.35, gripFromSupport(gripSupport, drive.minNormalY)),
      gripIn,
    });
  };

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
      pushHit(wheel, supportHit, false);
    } else if (wantProbe) {
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
    }

    // Front lip: while planted, grab steep face ahead for curb/ramp crest.
    if (wantProbe && wheel.steered && supportHit && supportHit.support > 0.8) {
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
      if (faceHit && faceHit.support < 0.6) pushHit(wheel, faceHit, true);
    }
  }

  if (contacts.length === 0) return;

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
  const driveScale =
    state.commandSpeed >= 0
      ? clampImpulse(frontFn / planted, 0, 1)
      : clampImpulse(rearFn / planted, 0, 1);
  const rollScale = rollAuthority(contacts, weight);
  const perWheelCap =
    (drive.maxAccel * Math.max(0.5, chassis.mass()) * dt * Math.max(0.45, driveScale) * 1.35) /
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
  // Idle: spheres + coilovers + links only — no Coulomb (avoids tip/orbit/self-push).
  if (!wantDrive) return;

  // Longitudinal motor at chassis COM (avoids soft-link soak / pitch reverse).
  // Coulomb at hubs owns lateral + reverse-yaw; spheres own vertical plant.
  {
    basis.set(0, 0, -1).applyQuaternion(q);
    const intoUp = basis.x * upx + basis.y * upy + basis.z * upz;
    let fx = basis.x - upx * intoUp;
    let fy = basis.y - upy * intoUp;
    let fz = basis.z - upz * intoUp;
    const fLen = Math.hypot(fx, fy, fz);
    if (fLen > 1e-5) {
      fx /= fLen;
      fy /= fLen;
      fz /= fLen;
      const vLong = lin.x * fx + lin.y * fy + lin.z * fz;
      const err = state.commandSpeed - vLong;
      const mass = Math.max(0.5, chassis.mass());
      // Planar COM assist for flat drive; hub long Coulomb along tx supplies climb.
      const motorCap = Math.min(
        drive.maxAccel * mass * dt * Math.max(0.35, driveScale) * 0.7,
        drive.maxForce * dt * 0.7
      );
      const j = clampImpulse(err * mass * 0.75 * dt, -motorCap, motorCap);
      chassis.applyImpulse({ x: fx * j, y: fy * j, z: fz * j }, true);

      // Climb boost along low-support contact tangents (uphill / vertical face).
      let clx = 0, cly = 0, clz = 0, nClimb = 0;
      for (const c of contacts) {
        if (c.ny < 0.9) {
          clx += c.tx; cly += c.ty; clz += c.tz; nClimb += 1;
        }
      }
      if (nClimb > 0 && Math.abs(state.commandSpeed) > 0.05) {
        clx /= nClimb; cly /= nClimb; clz /= nClimb;
        const cLen = Math.hypot(clx, cly, clz);
        if (cLen > 1e-5) {
          clx /= cLen; cly /= cLen; clz /= cLen;
          const vClimb = lin.x * clx + lin.y * cly + lin.z * clz;
          const climbErr = state.commandSpeed - vClimb;
          const climbCap = Math.min(
            drive.maxAccel * mass * dt * Math.max(0.4, driveScale) * 0.55,
            drive.maxForce * dt * 0.55
          );
          const jc = clampImpulse(climbErr * mass * 0.7 * dt, -climbCap, climbCap);
          chassis.applyImpulse({ x: clx * jc, y: cly * jc, z: clz * jc }, true);
        }
      }

      // Phase 6: mild drive-only upright restore (anti pitch-dive; idle path unchanged).
      if (upright < 0.9 && upright > 0.4) {
        basis.set(1, 0, 0).applyQuaternion(q);
        const torque = (0.9 - upright) * 0.85 * mass * dt;
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

  syncAxlePlanar(chassis, axles);

  const vel = chassis.linvel();
  const planar = Math.hypot(vel.x, vel.z);
  if (planar > rig.maxSpeed * 1.15) {
    const scale = (rig.maxSpeed * 1.15) / planar;
    chassis.setLinvel({ x: vel.x * scale, y: vel.y, z: vel.z * scale }, true);
    syncAxlePlanar(chassis, axles);
  }

}
