/**
 * Locked solid-axle drive on the soft tire patch.
 *
 * One spin speed per axle (spool). Motor torque is capped by driveTorque and by
 * the patch normal load, then longitudinal stiction kills slip so a planted
 * tire does not creep. Forces land on the axle at the hub; distance links carry
 * them to the chassis. A small wrap torque is all the housing is allowed to wind.
 *
 * The radial patch (tirePatch.ts) is what climbs a lip. There is no face-claw,
 * impact hold, or chassis linvel snap — those fought the rigid hub sphere.
 *
 * Next upgrades: node-ring carcass, open diff, portal gears.
 */
import type RAPIER from "@dimforge/rapier3d-compat";
import * as THREE from "three";
import type { DriveInput } from "../input/Input.ts";
import type { KitWheelDef, RigDef, WheelId } from "./types.ts";
import type { KitAxleRuntime } from "./kitBodies.ts";
import { axleQuaternion, hubWorldPosition } from "./kitBodies.ts";
import { clamp, invMassAlong } from "./impulse.ts";
import { applyTirePatch, PATCH_RAYS, type TirePatchHit } from "./tirePatch.ts";

const FRICTION_ITERS = 5;
const FRICTION_RELAX = 0.95;
/** Virtual spool inertia (kg·m²). Not the housing inertia. */
const SPIN_INERTIA = 0.02;
const MOTOR_KP = 160;
/** Safety only. Crawl speed lives in maxSpeed; this stops a solver blow-up. */
const EXPLODE_SPEED = 8;
const WRAP_FRACTION = 0.04;
const WRAP_TORQUE_CAP = 6;

const AXLE_YAW_K = 18;
const AXLE_YAW_C = 7.2;
const AXLE_YAW_TORQUE_CAP = 5;
const AXLE_YAW_SOFT = 0.09;
const AXLE_YAW_STABILIZE_LIMIT = 0.7;

export type TireContactSnapshot = {
  id: WheelId;
  axleId: string;
  deflection: number;
  nx: number;
  ny: number;
  nz: number;
  /** Post-solve longitudinal slip (m/s). Near zero when the tire is planted. */
  slip: number;
  omega: number;
  fn: number;
};

export type HubDriveState = {
  commandSpeed: number;
  steer: number;
  dragHold: boolean;
  omega: Map<string, number>;
  deflection: Map<string, number[]>;
  contacts: TireContactSnapshot[];
};

export function createHubDriveState(): HubDriveState {
  return {
    commandSpeed: 0,
    steer: 0,
    dragHold: false,
    omega: new Map(),
    deflection: new Map(),
    contacts: [],
  };
}

export function resetHubDriveState(state: HubDriveState): void {
  state.commandSpeed = 0;
  state.steer = 0;
  state.dragHold = false;
  state.omega.clear();
  state.deflection.clear();
  state.contacts.length = 0;
}

type WheelPlant = {
  def: KitWheelDef;
  axle: KitAxleRuntime;
  patch: TirePatchHit;
  hubX: number;
  hubY: number;
  hubZ: number;
  lx: number;
  ly: number;
  lz: number;
  sx: number;
  sy: number;
  sz: number;
  slip: number;
  accLong: number;
};

const q = new THREE.Quaternion();
const axis = new THREE.Vector3();
const hint = new THREE.Vector3();
const up = new THREE.Vector3();
const basis = new THREE.Vector3();
const basisUp = new THREE.Vector3();
const scratchForward = new THREE.Vector3();
const scratchRight = new THREE.Vector3();
const scratchLat = new THREE.Vector3();

function filtersFor(state: HubDriveState, wheelId: string): number[] {
  let row = state.deflection.get(wheelId);
  if (!row || row.length !== PATCH_RAYS) {
    row = new Array<number>(PATCH_RAYS).fill(0);
    state.deflection.set(wheelId, row);
  }
  return row;
}

/** maxSpeed is a planar cap so a ramp lip is not taken at a run. */
function capPlanarSpeed(body: RAPIER.RigidBody, maxSpeed: number): void {
  const vel = body.linvel();
  const planar = Math.hypot(vel.x, vel.z);
  let vy = vel.y;
  if (vy > 1.35) vy = 1.35;
  if (planar <= maxSpeed || planar < 1e-5) {
    if (vy !== vel.y) body.setLinvel({ x: vel.x, y: vy, z: vel.z }, true);
    return;
  }
  const scale = maxSpeed / planar;
  body.setLinvel({ x: vel.x * scale, y: vy, z: vel.z * scale }, true);
}

function explodeCap(body: RAPIER.RigidBody): void {
  const vel = body.linvel();
  const speed = Math.hypot(vel.x, vel.y, vel.z);
  if (speed > EXPLODE_SPEED && speed > 1e-6) {
    const scale = EXPLODE_SPEED / speed;
    body.setLinvel({ x: vel.x * scale, y: vel.y * scale, z: vel.z * scale }, true);
  }
}

function applyAxleYawDamping(
  chassis: RAPIER.RigidBody,
  axles: Map<string, KitAxleRuntime>,
  dt: number,
  steerTarget: number = 0
): void {
  if (!(dt > 0)) return;
  const cr = chassis.rotation();
  q.set(cr.x, cr.y, cr.z, cr.w);
  basisUp.set(0, 1, 0).applyQuaternion(q);
  scratchForward.set(1, 0, 0).applyQuaternion(q);
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
    const cos = clamp(basis.dot(lat), -1, 1);
    scratchRight.crossVectors(lat, basis);
    const sin = basisUp.dot(scratchRight);
    const yaw = Math.atan2(sin, cos);
    const desiredYaw = axle.id === "front" ? steerTarget : 0;
    const error = Math.atan2(Math.sin(yaw - desiredYaw), Math.cos(yaw - desiredYaw));
    const abs = Math.abs(error);
    // Keep front axles aligned to the commanded steer, but do not fight large
    // terrain-induced rotation if the driver is not actively steering.
    if (desiredYaw === 0 && abs > AXLE_YAW_STABILIZE_LIMIT) continue;
    const excess = Math.max(0, abs - AXLE_YAW_SOFT);
    const ang = axle.body.angvel();
    const omega = ang.x * basisUp.x + ang.y * basisUp.y + ang.z * basisUp.z;
    const sign = error >= 0 ? 1 : -1;
    const steerPull = desiredYaw !== 0 ? -error * AXLE_YAW_K : 0;
    const torque = clamp(
      steerPull + (excess > 0 ? -sign * AXLE_YAW_K * excess : 0) - AXLE_YAW_C * omega,
      -AXLE_YAW_TORQUE_CAP,
      AXLE_YAW_TORQUE_CAP
    );
    const j = torque * dt;
    axle.body.applyTorqueImpulse({ x: basisUp.x * j, y: basisUp.y * j, z: basisUp.z * j }, true);
  }
}

function projectTangent(
  nx: number,
  ny: number,
  nz: number,
  fx: number,
  fy: number,
  fz: number
): { x: number; y: number; z: number } | null {
  const into = fx * nx + fy * ny + fz * nz;
  const tx = fx - nx * into;
  const ty = fy - ny * into;
  const tz = fz - nz * into;
  const len = Math.hypot(tx, ty, tz);
  if (len < 1e-5) return null;
  return { x: tx / len, y: ty / len, z: tz / len };
}

function solveAxle(
  chassis: RAPIER.RigidBody,
  plants: WheelPlant[],
  omega: number,
  commandSpeed: number,
  driveTorque: number,
  mu: number,
  maxForce: number,
  maxAccel: number,
  dt: number
): number {
  const planted = plants.filter((p) => p.patch.fn > 0.4);
  const radius = plants[0]?.def.radius ?? 0.06;
  if (!(radius > 0)) return omega;

  // Share the crawl accel budget across four tires so a deflection spike cannot rip the links.
  const accelForce = (Math.max(0, maxAccel) * Math.max(chassis.mass(), 0.5)) / 4;
  let traction = 0;
  for (const plant of planted) {
    traction += Math.min(mu * plant.patch.fn, maxForce, accelForce) * radius;
  }
  if (traction <= 1e-4) {
    return omega * Math.exp(-dt / 0.35);
  }

  const tMax = Math.min(driveTorque, traction);
  const omegaCmd = commandSpeed / radius;
  const tMotor = clamp(MOTOR_KP * (omegaCmd - omega), -tMax, tMax);
  let w = omega + (tMotor / SPIN_INERTIA) * dt;

  for (let iter = 0; iter < FRICTION_ITERS; iter += 1) {
    for (const plant of planted) {
      const axleBody = plant.axle.body;
      const vel = chassis.linvel();
      const vLong = vel.x * plant.lx + vel.y * plant.ly + vel.z * plant.lz;
      const slip = vLong - w * radius;
      const invAxle = Math.max(axleBody.invMass(), 1e-8);
      const invChassis = Math.max(chassis.invMass(), 1e-8);
      // Rigid-link split: both bodies take the same Δv so the rod does not have to
      // ferry the drive force (a soft rod was eating it).
      const invCombined = 1 / (1 / invAxle + 1 / invChassis);
      const wSlip = invCombined + (radius * radius) / SPIN_INERTIA;
      const limit = Math.min(mu * plant.patch.fn, maxForce, accelForce) * dt;
      const j = clamp(
        (-slip / wSlip) * FRICTION_RELAX,
        -limit - plant.accLong,
        limit - plant.accLong
      );
      if (j !== 0) {
        const shareAxle = invCombined / invAxle;
        const shareChassis = invCombined / invChassis;
        axleBody.applyImpulse(
          { x: plant.lx * j * shareAxle, y: plant.ly * j * shareAxle, z: plant.lz * j * shareAxle },
          true
        );
        chassis.applyImpulse(
          {
            x: plant.lx * j * shareChassis,
            y: plant.ly * j * shareChassis,
            z: plant.lz * j * shareChassis,
          },
          true
        );
        w -= (j * radius) / SPIN_INERTIA;
        plant.accLong += j;
        const ar = axleBody.rotation();
        q.set(ar.x, ar.y, ar.z, ar.w);
        axis.set(1, 0, 0).applyQuaternion(q);
        const wrap = clamp(j * radius * WRAP_FRACTION, -WRAP_TORQUE_CAP * dt, WRAP_TORQUE_CAP * dt);
        axleBody.applyTorqueImpulse({ x: axis.x * wrap, y: axis.y * wrap, z: axis.z * wrap }, true);
      }

      const v2 = axleBody.velocityAtPoint({ x: plant.hubX, y: plant.hubY, z: plant.hubZ });
      const vLat = v2.x * plant.sx + v2.y * plant.sy + v2.z * plant.sz;
      const invLat = invMassAlong(axleBody, plant.hubX, plant.hubY, plant.hubZ, plant.sx, plant.sy, plant.sz);
      const used = Math.abs(plant.accLong);
      const latRoom = Math.sqrt(Math.max(0, limit * limit - used * used));
      const jLat = clamp((-vLat / invLat) * FRICTION_RELAX, -latRoom, latRoom);
      if (jLat !== 0) {
        axleBody.applyImpulseAtPoint(
          { x: plant.sx * jLat, y: plant.sy * jLat, z: plant.sz * jLat },
          { x: plant.hubX, y: plant.hubY, z: plant.hubZ },
          true
        );
      }
    }
  }

  for (const plant of plants) {
    const vel = chassis.linvel();
    const vLong = vel.x * plant.lx + vel.y * plant.ly + vel.z * plant.lz;
    plant.slip = plant.patch.fn > 0.4 ? vLong - w * radius : 0;
  }
  return w;
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
  state.contacts.length = 0;
  if (!(dt > 0) || wheels.length === 0) return;
  const kit = rig.kit;
  if (!kit) return;

  const drive = kit.drive;
  const tire = kit.tire;
  const idleEps = drive.idleThrottleEps ?? 0.04;
  const idleEnterSpeed = drive.idleEnterSpeed ?? 0.03;
  const idleExitSpeed = drive.idleExitSpeed ?? 0.07;
  const idlePlanarEnterSpeed = drive.idlePlanarEnterSpeed ?? 0.12;
  const idlePlanarExitSpeed = drive.idlePlanarExitSpeed ?? 0.24;
  const planarSpeed = Math.hypot(chassis.linvel().x, chassis.linvel().z);

  if (state.dragHold) {
    if (
      Math.abs(input.throttle) > idleEps ||
      Math.abs(state.commandSpeed) > idleExitSpeed ||
      planarSpeed > idlePlanarExitSpeed
    ) {
      state.dragHold = false;
    }
  } else if (
    Math.abs(input.throttle) <= idleEps &&
    Math.abs(state.commandSpeed) <= idleEnterSpeed &&
    planarSpeed <= idlePlanarEnterSpeed
  ) {
    state.dragHold = true;
  }

  if (drive.steerRate && drive.steerRate > 0) {
    const steerStep = drive.steerRate * dt;
    state.steer += clamp(input.steer - state.steer, -steerStep, steerStep);
  } else {
    const blend = 1 - Math.exp(-dt / 0.08);
    state.steer += (input.steer - state.steer) * blend;
  }
  if (input.steer === 0 && Math.abs(state.steer) < 0.02) state.steer = 0;

  // Keep the turn assist subtle: reduce straight-line momentum only a little so the
  // chassis naturally yaws without feeling like a forced point-turn or a visible speed brake.
  const steerAmount = Math.abs(state.steer);
  const turnDrag = 1 - Math.min(0.12, steerAmount * 0.1);
  const idleTorqueScale = drive.idleTorqueScale ?? 0.28;
  const idleForceScale = drive.idleForceScale ?? 0.34;
  const idleMuScale = drive.idleMuScale ?? 0.72;
  const idleAccelScale = drive.idleAccelScale ?? 0.4;
  let desired = input.throttle * rig.maxSpeed * turnDrag;
  let driveTorque = drive.driveTorque;
  let maxForce = drive.maxForce;
  let mu = drive.mu;
  let maxAccel = drive.maxAccel;

  if (state.dragHold) {
    desired = 0;
    state.commandSpeed = 0;
    driveTorque *= idleTorqueScale;
    maxForce *= idleForceScale;
    mu *= idleMuScale;
    maxAccel *= idleAccelScale;
  } else {
    const slew = drive.maxAccel * dt;
    state.commandSpeed += clamp(desired - state.commandSpeed, -slew, slew);
  }

  const rot = chassis.rotation();
  q.set(rot.x, rot.y, rot.z, rot.w);
  up.set(0, 1, 0).applyQuaternion(q);
  const gravity = world.gravity;
  const gMag = Math.hypot(gravity.x, gravity.y, gravity.z) || 9.81;
  const upright = up.x * -gravity.x / gMag + up.y * -gravity.y / gMag + up.z * -gravity.z / gMag;
  const reverseSign = input.throttle < 0 ? -1 : 1;
  applyAxleYawDamping(chassis, axles, dt, state.steer * drive.steerAngle * reverseSign);

  const plants: WheelPlant[] = [];
  for (const wheel of wheels) {
    const hub = hubWorldPosition(wheel.axle, wheel.def.hubOffset);
    const steer = wheel.def.steered ? state.steer * drive.steerAngle : 0;
    axis.set(1, 0, 0).applyQuaternion(axleQuaternion(wheel.axle));
    hint.set(0, 0, -1).applyQuaternion(q);
    if (steer !== 0) {
      axis.applyAxisAngle(up, steer);
      hint.applyAxisAngle(up, steer);
    }
    const patch = applyTirePatch(
      world,
      wheel.axle.body,
      hub.x,
      hub.y,
      hub.z,
      axis.x,
      axis.y,
      axis.z,
      hint.x,
      hint.y,
      hint.z,
      wheel.def.radius,
      tire,
      filtersFor(state, wheel.def.id),
      dt
    );
    // World-horizontal heading. A chassis-up tangent points up the nose and launches.
    const tangent = projectTangent(0, 1, 0, hint.x, hint.y, hint.z);
    const lx = tangent?.x ?? hint.x;
    const ly = tangent?.y ?? hint.y;
    const lz = tangent?.z ?? hint.z;
    let sx = patch.ny * lz - patch.nz * ly;
    let sy = patch.nz * lx - patch.nx * lz;
    let sz = patch.nx * ly - patch.ny * lx;
    const sLen = Math.hypot(sx, sy, sz);
    if (sLen > 1e-5) {
      sx /= sLen;
      sy /= sLen;
      sz /= sLen;
    } else {
      sx = 1;
      sy = 0;
      sz = 0;
    }
    plants.push({
      def: wheel.def,
      axle: wheel.axle,
      patch,
      hubX: hub.x,
      hubY: hub.y,
      hubZ: hub.z,
      lx,
      ly,
      lz,
      sx,
      sy,
      sz,
      slip: 0,
      accLong: 0,
    });
  }

  const driveOn = upright >= drive.minUpright;
  const byAxle = new Map<string, WheelPlant[]>();
  for (const plant of plants) {
    const list = byAxle.get(plant.axle.id) ?? [];
    list.push(plant);
    byAxle.set(plant.axle.id, list);
  }
  for (const [axleId, group] of byAxle) {
    const prev = state.omega.get(axleId) ?? 0;
    const wantDrive = state.dragHold || Math.abs(state.commandSpeed) > 0.08;
    const next = driveOn && wantDrive
      ? solveAxle(
          chassis,
          group,
          prev,
          state.commandSpeed,
          driveTorque,
          mu,
          maxForce,
          maxAccel,
          dt
        )
      : prev * Math.exp(-dt / 0.35);
    state.omega.set(axleId, next);
  }

  for (const plant of plants) {
    const omega = state.omega.get(plant.axle.id) ?? 0;
    state.contacts.push({
      id: plant.def.id,
      axleId: plant.axle.id,
      deflection: plant.patch.deflection,
      nx: plant.patch.nx,
      ny: plant.patch.ny,
      nz: plant.patch.nz,
      slip: plant.slip,
      omega,
      fn: plant.patch.fn,
    });
  }

  capPlanarSpeed(chassis, rig.maxSpeed);
  if (state.dragHold) {
    const vel = chassis.linvel();
    const planar = Math.hypot(vel.x, vel.z);
    if (planar < idlePlanarExitSpeed) {
      const damp = Math.max(0, 1 - dt * 12);
      chassis.setLinvel({ x: vel.x * damp, y: vel.y, z: vel.z * damp }, true);
      if (planar < 0.02) {
        chassis.setLinvel({ x: 0, y: vel.y, z: 0 }, true);
      }
    } else {
      state.dragHold = false;
    }
  }
  if (Math.abs(state.commandSpeed) > 0.08 && Math.abs(state.steer) < 0.2) {
    const capped = chassis.linvel();
    for (const axle of axles.values()) {
      const av = axle.body.linvel();
      // While driving, keep the housing from lagging in the plane. Heave stays free.
      axle.body.setLinvel({ x: capped.x, y: av.y, z: capped.z }, true);
    }
  }
  explodeCap(chassis);
}
