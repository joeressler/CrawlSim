/**
 * Per-wheel suspension ray and Coulomb friction.
 * Each step casts chassis-down from the hardpoint, pushes along the contact
 * normal with a spring and damper, then clamps tangent grip to μ·Fn.
 * Throttle slews a longitudinal target speed; `maxAccel` and `driveTorque / radius` cap the force.
 * Drive fades when the axle that would lift is unloaded, so the steering tires stay down.
 * Contacts whose normal is too steep are ignored so a ledge face cannot grab a tire.
 * Unloaded tires also probe a short lip cast beside the tread so a hanging wheel finds
 * a ledge top before the visual cylinder clips through it.
 * Steering rotates the front tire basis. Lateral grip yaws the chassis and
 * pulls its velocity along that heading, forward or reverse. The roll part of
 * that force is cancelled so a steer cannot lift a tire. It fades if a side
 * unloads or fewer than three tires are down.
 * Extension is clamped to restLength and compression to restLength − maxTravel along
 * the chassis-down strut. If the chassis is upside-down past minUpright, tires unload.
 * Kit rigs do not use this path. Locked-axle crawl lives in kitHubDrive.ts /
 * tirePatch.ts. Next upgrades there: node-ring carcass, open diff, portal gears.
 * Legacy next step if this path returns: wheel joints, then lockers.
 */
import RAPIER from "@dimforge/rapier3d-compat";
import * as THREE from "three";
import type { DriveInput } from "../input/Input.ts";
import type { RigDef } from "./types.ts";

const FRICTION_ITERATIONS = 6;
const FRICTION_RELAX = 0.35;
/** How fast the ray length may shorten, so a bump cannot spike one spring. */
const MAX_COMPRESS_RATE = 1.1;
/** How fast the ray length may lengthen when a tire drops. */
const MAX_EXTEND_RATE = 4;
/** Fraction of the steer roll moment left in place. The rest is cancelled so the rig turns without lifting a tire. */
const ROLL_KEEP = 0.18;

export type WheelSim = {
  offset: { x: number; y: number; z: number };
  radius: number;
  driven: boolean;
  steered: boolean;
  restLength: number;
  springK: number;
  damperC: number;
  mu: number;
  /** Ray hit distance; `restLength` when the tire is unloaded. */
  suspensionLength: number;
  /** Visual roll, radians. */
  spin: number;
  /** Visual steer, radians. */
  steer: number;
  /** World-space direction the tire visual hangs, unit length. */
  castX: number;
  castY: number;
  castZ: number;
  /** 0 when the tire just found ground, 1 after it has settled. */
  plant: number;
};

export type DriveState = {
  commandSpeed: number;
  /** Smoothed steer command, -1..1. */
  steer: number;
};

export function createDriveState(): DriveState {
  return { commandSpeed: 0, steer: 0 };
}

export function resetDriveState(state: DriveState): void {
  state.commandSpeed = 0;
  state.steer = 0;
}

type WheelContact = {
  sim: WheelSim;
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
  /** Chassis forward on the contact plane, before steer. */
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
  /** 0 on a ledge lip, 1 on flat support. Scales μ. */
  grip: number;
  /** How settled this tire is. Fresh contacts stay soft so they do not bounce off a lip. */
  gripIn: number;
};

const q = new THREE.Quaternion();
const basis = new THREE.Vector3();
const basisUp = new THREE.Vector3();
const scratchQuat = new THREE.Quaternion();
const scratchForward = new THREE.Vector3();

export function createWheelSims(rig: RigDef): WheelSim[] {
  const shared = rig.suspension;
  return rig.wheels.map((wheel) => ({
    offset: wheel.offset,
    radius: wheel.radius,
    driven: wheel.driven,
    steered: wheel.steered,
    restLength: wheel.restLength ?? shared.restLength,
    springK: wheel.springK ?? shared.springK,
    damperC: wheel.damperC ?? shared.damperC,
    mu: wheel.mu ?? shared.mu,
    suspensionLength: wheel.restLength ?? shared.restLength,
    spin: 0,
    steer: 0,
    castX: 0,
    castY: -1,
    castZ: 0,
    plant: 0,
  }));
}

export function resetWheelSims(wheels: WheelSim[]): void {
  for (const wheel of wheels) {
    wheel.suspensionLength = wheel.restLength;
    wheel.spin = 0;
    wheel.steer = 0;
    wheel.castX = 0;
    wheel.castY = -1;
    wheel.castZ = 0;
    wheel.plant = 0;
  }
}

type SupportHit = {
  dist: number;
  nx: number;
  ny: number;
  nz: number;
  dx: number;
  dy: number;
  dz: number;
  support: number;
};

function castSupport(
  world: RAPIER.World,
  ox: number,
  oy: number,
  oz: number,
  dx: number,
  dy: number,
  dz: number,
  maxToi: number,
  chassis: RAPIER.RigidBody,
  gx: number,
  gy: number,
  gz: number,
  minNormalY: number
): SupportHit | null {
  const dirLen = Math.hypot(dx, dy, dz) || 1;
  const ux = dx / dirLen;
  const uy = dy / dirLen;
  const uz = dz / dirLen;
  const ray = new RAPIER.Ray({ x: ox, y: oy, z: oz }, { x: ux, y: uy, z: uz });
  const hit = world.castRayAndGetNormal(ray, maxToi, false, undefined, undefined, undefined, chassis);
  if (!hit) return null;
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
  if (support < minNormalY) return null;
  return { dist: hit.timeOfImpact, nx, ny, nz, dx: ux, dy: uy, dz: uz, support };
}

/** Probe beside the tread so a hanging tire finds a ledge top before the cylinder clips it. */
function castLipAssist(
  world: RAPIER.World,
  hx: number,
  hy: number,
  hz: number,
  dx: number,
  dy: number,
  dz: number,
  upx: number,
  upy: number,
  upz: number,
  quat: THREE.Quaternion,
  wheel: WheelSim,
  steer: number,
  chassis: RAPIER.RigidBody,
  gx: number,
  gy: number,
  gz: number,
  minNormalY: number
): SupportHit | null {
  const dirLen = Math.hypot(dx, dy, dz) || 1;
  const ux = dx / dirLen;
  const uy = dy / dirLen;
  const uz = dz / dirLen;
  const r = wheel.radius;
  const lift = 0.06;
  const reach = r * 1.05;
  const side = Math.sign(wheel.offset.x) || 1;
  basis.set(side, 0, 0).applyQuaternion(quat);
  const sx = basis.x;
  const sy = basis.y;
  const sz = basis.z;
  scratchForward.set(0, 0, -1).applyQuaternion(quat);
  if (steer !== 0) scratchForward.applyAxisAngle(basisUp.set(0, 1, 0).applyQuaternion(quat), steer);
  const offsets = [
    { x: sx * reach, y: sy * reach, z: sz * reach },
    { x: -sx * reach, y: -sy * reach, z: -sz * reach },
    { x: scratchForward.x * reach, y: scratchForward.y * reach, z: scratchForward.z * reach },
    { x: -scratchForward.x * reach, y: -scratchForward.y * reach, z: -scratchForward.z * reach },
    { x: (sx + scratchForward.x) * reach * 0.7, y: (sy + scratchForward.y) * reach * 0.7, z: (sz + scratchForward.z) * reach * 0.7 },
    { x: (-sx + scratchForward.x) * reach * 0.7, y: (-sy + scratchForward.y) * reach * 0.7, z: (-sz + scratchForward.z) * reach * 0.7 },
  ];
  let best: SupportHit | null = null;
  for (const offset of offsets) {
    const ox = hx + offset.x + upx * lift;
    const oy = hy + offset.y + upy * lift;
    const oz = hz + offset.z + upz * lift;
    const hit = castSupport(world, ox, oy, oz, ux, uy, uz, wheel.restLength + lift, chassis, gx, gy, gz, minNormalY);
    if (!hit) continue;
    const hxHit = ox + ux * hit.dist;
    const hyHit = oy + uy * hit.dist;
    const hzHit = oz + uz * hit.dist;
    const fromHub = (hxHit - hx) * ux + (hyHit - hy) * uy + (hzHit - hz) * uz;
    if (fromHub < 0.02 || fromHub > wheel.restLength) continue;
    const candidate: SupportHit = {
      dist: fromHub,
      nx: hit.nx,
      ny: hit.ny,
      nz: hit.nz,
      dx: ux,
      dy: uy,
      dz: uz,
      support: hit.support,
    };
    best = preferHigher(hx, hy, hz, best, candidate);
  }
  return best;
}

function preferHigher(
  _hx: number,
  hy: number,
  _hz: number,
  a: SupportHit | null,
  b: SupportHit | null
): SupportHit | null {
  if (!a) return b;
  if (!b) return a;
  const ay = hy + a.dy * a.dist;
  const by = hy + b.dy * b.dist;
  // Prefer the contact that keeps the tire higher (less penetration through a lip).
  if (by > ay + 0.01) return b;
  if (ay > by + 0.01) return a;
  return a.dist <= b.dist ? a : b;
}

/** Map a world hit back onto the chassis-down strut length from the hub. */
function projectOntoStrut(
  hx: number,
  hy: number,
  hz: number,
  hit: SupportHit,
  dx: number,
  dy: number,
  dz: number
): number | null {
  const px = hx + hit.dx * hit.dist;
  const py = hy + hit.dy * hit.dist;
  const pz = hz + hit.dz * hit.dist;
  const along = (px - hx) * dx + (py - hy) * dy + (pz - hz) * dz;
  if (along < 0.02) return null;
  return along;
}

export function applyDrive(
  world: RAPIER.World,
  chassis: RAPIER.RigidBody,
  wheels: WheelSim[],
  rig: RigDef,
  input: DriveInput,
  state: DriveState,
  dt: number
): void {
  if (dt <= 0 || wheels.length === 0) return;

  const desiredSpeed = input.throttle * rig.maxSpeed;
  const speedStep = rig.suspension.maxAccel * dt;
  state.commandSpeed += clampImpulse(desiredSpeed - state.commandSpeed, -speedStep, speedStep);
  const steerBlend = 1 - Math.exp(-dt / 0.08);
  state.steer += (input.steer - state.steer) * steerBlend;
  if (input.steer === 0 && Math.abs(state.steer) < 0.02) state.steer = 0;
  const steerCmd = state.steer;

  const rotation = chassis.rotation();
  q.set(rotation.x, rotation.y, rotation.z, rotation.w);
  const origin = chassis.translation();
  const ox = origin.x;
  const oy = origin.y;
  const oz = origin.z;

  basis.set(0, -1, 0).applyQuaternion(q);
  const dx = basis.x;
  const dy = basis.y;
  const dz = basis.z;
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
  if (upright < rig.suspension.minUpright) {
    for (const wheel of wheels) {
      wheel.suspensionLength = wheel.restLength;
      wheel.plant = 0;
      wheel.castX = dx;
      wheel.castY = dy;
      wheel.castZ = dz;
    }
    return;
  }

  const contacts: WheelContact[] = [];

  for (const wheel of wheels) {
    const minLength = Math.max(0.05, wheel.restLength - rig.suspension.maxTravel);
    const steer = wheel.steered ? steerCmd * rig.suspension.steerAngle : 0;
    wheel.steer = steer;

    basis.set(wheel.offset.x, wheel.offset.y, wheel.offset.z).applyQuaternion(q);
    const hx = ox + basis.x;
    const hy = oy + basis.y;
    const hz = oz + basis.z;

    // Strut axis is chassis-down only. World-down / lip assists are only for finding
    // the same support surface while the chassis is still mostly upright.
    let probed = castSupport(
      world,
      hx,
      hy,
      hz,
      dx,
      dy,
      dz,
      wheel.restLength,
      chassis,
      gx,
      gy,
      gz,
      rig.suspension.minNormalY
    );
    scratchForward.set(0, 0, -1).applyQuaternion(q);
    const noseDown = -(scratchForward.x * upx + scratchForward.y * upy + scratchForward.z * upz);
    let immediate = false;
    if (upright > 0.45) {
      const downx = -upx;
      const downy = -upy;
      const downz = -upz;
      if (!probed && noseDown > 0.5) {
        const lift = 0.08;
        const caught = castSupport(
          world,
          hx + scratchForward.x * wheel.radius * 0.85 + upx * lift,
          hy + scratchForward.y * wheel.radius * 0.85 + upy * lift,
          hz + scratchForward.z * wheel.radius * 0.85 + upz * lift,
          downx,
          downy,
          downz,
          wheel.restLength + lift,
          chassis,
          gx,
          gy,
          gz,
          rig.suspension.minNormalY
        );
        if (caught) {
          const fromHub = projectOntoStrut(hx, hy, hz, caught, dx, dy, dz);
          if (fromHub !== null) {
            probed = { ...caught, dist: fromHub, dx, dy, dz };
            immediate = true;
          }
        }
      }
      const lip = castLipAssist(
        world,
        hx,
        hy,
        hz,
        downx,
        downy,
        downz,
        upx,
        upy,
        upz,
        q,
        wheel,
        steer,
        chassis,
        gx,
        gy,
        gz,
        rig.suspension.minNormalY
      );
      if (lip) {
        const fromHub = projectOntoStrut(hx, hy, hz, lip, dx, dy, dz);
        if (fromHub !== null) {
          const strutHit: SupportHit = { ...lip, dist: fromHub, dx, dy, dz };
          if (!probed || strutHit.dist < probed.dist) probed = strutHit;
        }
      }
    }

    if (!probed) {
      wheel.suspensionLength = wheel.restLength;
      wheel.plant = 0;
      wheel.castX = dx;
      wheel.castY = dy;
      wheel.castZ = dz;
      if (wheel.driven && input.throttle !== 0) {
        wheel.spin += (input.throttle * rig.maxSpeed * dt) / wheel.radius;
      }
      continue;
    }

    // Contact distance along the strut only, clamped to [bump stop, rest].
    const dist = clampImpulse(probed.dist, minLength, wheel.restLength);
    const nx = probed.nx;
    const ny = probed.ny;
    const nz = probed.nz;
    const rdx = dx;
    const rdy = dy;
    const rdz = dz;
    const support = probed.support;

    const previousLength = wheel.suspensionLength;
    const wasAir = wheel.plant < 0.05 || previousLength >= wheel.restLength - 0.01;
    const leftSurface = !immediate && !wasAir && previousLength < wheel.restLength - 0.02 && dist > previousLength + 0.045;
    const usedDist = clampImpulse(
      immediate || wasAir
        ? dist
        : dist < previousLength
          ? Math.max(dist, previousLength - MAX_COMPRESS_RATE * dt)
          : Math.min(dist, previousLength + MAX_EXTEND_RATE * dt),
      minLength,
      wheel.restLength
    );
    wheel.suspensionLength = usedDist;
    wheel.castX = rdx;
    wheel.castY = rdy;
    wheel.castZ = rdz;
    if (leftSurface) continue;

    const hardVel = chassis.velocityAtPoint({ x: hx, y: hy, z: hz });
    // Positive when the hardpoint moves along the outward normal (suspension extending).
    const closingSpeed = hardVel.x * nx + hardVel.y * ny + hardVel.z * nz;
    const spring = wheel.springK * (wheel.restLength - usedDist);
    const damp = wasAir && closingSpeed < 0 ? 0 : wheel.damperC * closingSpeed;
    const fnRaw = spring - damp;
    const fnCap = immediate ? chassis.mass() * gMag * 1.35 : (3 * chassis.mass() * gMag) / wheels.length;
    const gripIn = immediate ? 1 : wasAir ? 0.45 : 0.45 + 0.55 * wheel.plant;
    const fn = Math.min(Math.max(fnRaw, 0), fnCap) * gripIn;
    if (fn <= 0) {
      wheel.plant = 0;
      if (wheel.driven && input.throttle !== 0) {
        wheel.spin += (input.throttle * rig.maxSpeed * dt) / wheel.radius;
      }
      continue;
    }
    wheel.plant = Math.min(1, wheel.plant + dt / 0.14);

    const px = hx + rdx * usedDist;
    const py = hy + rdy * usedDist;
    const pz = hz + rdz * usedDist;
    chassis.applyImpulseAtPoint(
      { x: nx * fn * dt, y: ny * fn * dt, z: nz * fn * dt },
      { x: px, y: py, z: pz },
      true
    );

    basis.set(0, 0, -1).applyQuaternion(q);
    const chassisInto = basis.x * nx + basis.y * ny + basis.z * nz;
    let cx = basis.x - nx * chassisInto;
    let cy = basis.y - ny * chassisInto;
    let cz = basis.z - nz * chassisInto;
    const cLen = Math.hypot(cx, cy, cz);
    if (cLen < 1e-5) continue;
    cx /= cLen;
    cy /= cLen;
    cz /= cLen;
    if (steer !== 0) basis.applyAxisAngle(basisUp, steer);
    const into = basis.x * nx + basis.y * ny + basis.z * nz;
    let tx = basis.x - nx * into;
    let ty = basis.y - ny * into;
    let tz = basis.z - nz * into;
    const tLen = Math.hypot(tx, ty, tz);
    if (tLen < 1e-5) continue;
    tx /= tLen;
    ty /= tLen;
    tz /= tLen;
    let sx = ny * tz - nz * ty;
    let sy = nz * tx - nx * tz;
    let sz = nx * ty - ny * tx;
    const sLen = Math.hypot(sx, sy, sz);
    if (sLen < 1e-5) continue;
    sx /= sLen;
    sy /= sLen;
    sz /= sLen;

    contacts.push({
      sim: wheel,
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
      grip: gripFromSupport(support, rig.suspension.minNormalY),
      gripIn,
    });
  }

  const weight = chassis.mass() * gMag;
  let frontFn = 0;
  let rearFn = 0;
  let drivenContacts = 0;
  for (const contact of contacts) {
    if (contact.sim.driven) drivenContacts += 1;
    if (contact.sim.steered) frontFn += contact.fn;
    else rearFn += contact.fn;
  }
  const planted = 0.32 * weight;
  const driveScale = state.commandSpeed >= 0 ? clampImpulse(frontFn / planted, 0, 1) : clampImpulse(rearFn / planted, 0, 1);
  const perWheelCap =
    (rig.suspension.maxAccel * chassis.mass() * dt * driveScale) / Math.max(1, drivenContacts);
  const rollScale = rollAuthority(contacts, weight);
  basis.set(0, 0, -1).applyQuaternion(q);
  const lin = chassis.linvel();
  const fwdSpeed = lin.x * basis.x + lin.y * basis.y + lin.z * basis.z;
  const travel = Math.abs(fwdSpeed) > 0.25 ? Math.sign(fwdSpeed) : Math.sign(state.commandSpeed) || 1;

  for (let iteration = 0; iteration < FRICTION_ITERATIONS; iteration += 1) {
    for (const contact of contacts) {
      solveContact(chassis, contact, rig, state.commandSpeed, steerCmd, perWheelCap, rollScale, travel < 0, gx, gy, gz, dt);
    }
  }
  if (travel < 0) applyReverseYaw(chassis, rig, steerCmd, fwdSpeed, wheelbaseOf(wheels), gx, gy, gz, gMag, rollScale);
  else applySteer(chassis, contacts, rig, steerCmd, dt, rollScale);

  for (const contact of contacts) {
    const vel = chassis.velocityAtPoint({ x: contact.px, y: contact.py, z: contact.pz });
    const vLong = vel.x * contact.tx + vel.y * contact.ty + vel.z * contact.tz;
    contact.sim.spin += (vLong / contact.sim.radius) * dt;
  }

  const vel = chassis.linvel();
  const planar = Math.hypot(vel.x, vel.z);
  if (planar > rig.maxSpeed) {
    const scale = rig.maxSpeed / planar;
    chassis.setLinvel({ x: vel.x * scale, y: vel.y, z: vel.z * scale }, true);
  }
  dampSteepPitch(chassis, q, gx, gy, gz);
}

function solveContact(
  chassis: RAPIER.RigidBody,
  contact: WheelContact,
  rig: RigDef,
  commandSpeed: number,
  steerInput: number,
  perWheelCap: number,
  rollScale: number,
  reversing: boolean,
  gx: number,
  gy: number,
  gz: number,
  dt: number
): void {
  const { sim } = contact;
  const slip = tangentVelocity(chassis, contact, gx, gy, gz, dt);
  const driveStraight = reversing && sim.steered;
  const vDrive = driveStraight ? slip.vChassis : slip.vLong;
  const targetLong = sim.driven ? commandSpeed : vDrive;
  const steerShare = sim.steered && steerInput !== 0 && !reversing ? 0.45 : 0;
  const maxFric = frictionLimit(contact, rig.suspension.maxForce) * dt * (1 - steerShare);
  const motor = Math.min((rig.suspension.driveTorque / sim.radius) * dt, perWheelCap);
  const driveInv = driveStraight ? contact.invChassis : contact.invLong;
  const wantLong = clampImpulse(
    (-(vDrive - targetLong) / driveInv) * FRICTION_RELAX,
    -motor - contact.accLong,
    motor - contact.accLong
  );
  const latGrip = (reversing && sim.steered ? 0.25 : 1) * contact.gripIn;
  const latRoom = maxFric * Math.max(0, rollScale) * latGrip;
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

function dampSteepPitch(chassis: RAPIER.RigidBody, quat: THREE.Quaternion, gx: number, gy: number, gz: number): void {
  const gMag = Math.hypot(gx, gy, gz) || 9.81;
  const upx = -gx / gMag;
  const upy = -gy / gMag;
  const upz = -gz / gMag;
  scratchForward.set(0, 1, 0).applyQuaternion(quat);
  const level = scratchForward.x * upx + scratchForward.y * upy + scratchForward.z * upz;
  if (level > 0.62) return;
  basis.set(1, 0, 0).applyQuaternion(quat);
  const ang = chassis.angvel();
  const pitch = ang.x * basis.x + ang.y * basis.y + ang.z * basis.z;
  const inv = yawInvInertia(chassis, basis.x, basis.y, basis.z);
  const impulse = clampImpulse((-pitch * 0.45) / inv, -0.35, 0.35);
  if (impulse === 0) return;
  chassis.applyTorqueImpulse({ x: basis.x * impulse, y: basis.y * impulse, z: basis.z * impulse }, true);
}

function wheelbaseOf(wheels: WheelSim[]): number {
  let front = 0;
  let rear = 0;
  let frontCount = 0;
  let rearCount = 0;
  for (const wheel of wheels) {
    if (wheel.steered) {
      front += wheel.offset.z;
      frontCount += 1;
    } else {
      rear += wheel.offset.z;
      rearCount += 1;
    }
  }
  if (frontCount === 0 || rearCount === 0) return 0.9;
  return Math.max(0.4, Math.abs(front / frontCount - rear / rearCount));
}

/** Yaw about the up axis so reverse steer swings the tail instead of sliding along the tires. */
function applyReverseYaw(
  chassis: RAPIER.RigidBody,
  rig: RigDef,
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
  const delta = steerInput * rig.suspension.steerAngle;
  const speed = Math.min(fwdSpeed, -0.4);
  const omegaTarget = (speed / wheelbase) * Math.tan(delta);
  const ang = chassis.angvel();
  const ax = ang.x;
  const ay = ang.y;
  const az = ang.z;
  const upx = -gx / gMag;
  const upy = -gy / gMag;
  const upz = -gz / gMag;
  const omega = ax * upx + ay * upy + az * upz;
  const inv = yawInvInertia(chassis, upx, upy, upz);
  const correct = clampImpulse(((omegaTarget - omega) * 0.28) / inv, -0.22, 0.22) * rollScale;
  if (correct === 0) return;
  chassis.applyTorqueImpulse({ x: upx * correct, y: upy * correct, z: upz * correct }, true);
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

/** Extra lateral force so the steered tires pull the chassis along their heading while driving forward. */
function applySteer(
  chassis: RAPIER.RigidBody,
  contacts: WheelContact[],
  rig: RigDef,
  steerInput: number,
  dt: number,
  rollScale: number
): void {
  if (steerInput === 0 || rollScale <= 0) return;
  for (const contact of contacts) {
    if (!contact.sim.steered) continue;
    const cap = frictionLimit(contact, rig.suspension.maxForce) * dt * rollScale * contact.gripIn;
    const used = Math.hypot(contact.accLong, contact.accLat);
    const room = Math.sqrt(Math.max(0, cap * cap - used * used));
    const jLat = clampImpulse(steerInput * cap * 0.1, -room, room);
    applyLateral(chassis, contact, contact.sx, contact.sy, contact.sz, jLat);
    contact.accLat += jLat;
  }
}

function rollAuthority(contacts: WheelContact[], weight: number): number {
  if (contacts.length < 3) return 0;
  let leftFn = 0;
  let rightFn = 0;
  for (const contact of contacts) {
    if (contact.sim.offset.x <= 0) leftFn += contact.fn;
    else rightFn += contact.fn;
  }
  const sidePlant = 0.18 * weight;
  return clampImpulse(Math.min(leftFn, rightFn) / sidePlant, 0, 1);
}

function applyLateral(
  chassis: RAPIER.RigidBody,
  contact: WheelContact,
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
  chassis.applyTorqueImpulse({ x: -scratchForward.x * cancel, y: -scratchForward.y * cancel, z: -scratchForward.z * cancel }, true);
}

function applyAxisImpulse(
  chassis: RAPIER.RigidBody,
  contact: WheelContact,
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

function frictionLimit(contact: WheelContact, maxForce: number): number {
  return Math.min(contact.sim.mu * contact.grip * contact.fn, maxForce);
}

function gripFromSupport(support: number, minNormalY: number): number {
  const full = Math.max(minNormalY + 0.35, minNormalY);
  return clampImpulse((support - minNormalY) / (full - minNormalY), 0, 1);
}

function tangentVelocity(
  chassis: RAPIER.RigidBody,
  contact: WheelContact,
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

function clampImpulse(value: number, min: number, max: number): number {
  if (!Number.isFinite(value)) return 0;
  if (max < min) return 0;
  return Math.min(max, Math.max(min, value));
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
