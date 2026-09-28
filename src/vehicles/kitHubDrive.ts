/**
 * Phase 3 hub drive: longitudinal + steer at planted hubs.
 * Apply impulses to CHASSIS (and matching axle delta-v) so soft sphericals are not sheared.
 * Plant gate only — no stacked spring with chassis-ray.
 */
import RAPIER from "@dimforge/rapier3d-compat";
import * as THREE from "three";
import type { DriveInput } from "../input/Input.ts";
import type { KitWheelDef, RigDef, Vec3 } from "./types.ts";
import type { KitAxleRuntime } from "./kitBodies.ts";
import { hubWorldPosition } from "./kitBodies.ts";

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
};

const q = new THREE.Quaternion();
const forward = new THREE.Vector3();
const up = new THREE.Vector3();
const side = new THREE.Vector3();

function clamp(v: number, lo: number, hi: number): number {
  return Math.min(hi, Math.max(lo, v));
}

function hubPlanted(
  world: RAPIER.World,
  axle: KitAxleRuntime,
  hubOffset: Vec3,
  radius: number
): boolean {
  const hub = hubWorldPosition(axle, hubOffset);
  // Clearance gate: reliable on flat ground (ray TOI from hub+eps was stricter than plant).
  const clearance = hub.y - radius;
  if (clearance <= 0.025) return true;
  const ray = new RAPIER.Ray({ x: hub.x, y: hub.y, z: hub.z }, { x: 0, y: -1, z: 0 });
  const hit = world.castRay(ray, radius * 1.5, true, undefined, undefined, undefined, axle.body);
  if (!hit) return false;
  return hit.timeOfImpact <= radius * 1.2;
}

export function applyHubDrive(
  world: RAPIER.World,
  chassis: RAPIER.RigidBody,
  axles: Map<string, KitAxleRuntime>,
  wheels: HubWheel[],
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
  state.commandSpeed += clamp(desired - state.commandSpeed, -step, step);
  const blend = 1 - Math.exp(-dt / 0.08);
  state.steer += (input.steer - state.steer) * blend;
  if (input.steer === 0 && Math.abs(state.steer) < 0.02) state.steer = 0;

  const rot = chassis.rotation();
  q.set(rot.x, rot.y, rot.z, rot.w);
  forward.set(0, 0, -1).applyQuaternion(q);
  up.set(0, 1, 0).applyQuaternion(q);

  if (up.y < drive.minUpright) return;

  const wantDrive =
    Math.abs(state.commandSpeed) > 0.02 ||
    Math.abs(state.steer) > 0.02 ||
    input.throttle !== 0 ||
    input.steer !== 0;
  if (!wantDrive) return;

  const plantedWheels: HubWheel[] = [];
  for (const wheel of wheels) {
    if (hubPlanted(world, wheel.axle, wheel.def.hubOffset, wheel.def.radius)) {
      plantedWheels.push(wheel);
    }
  }
  if (plantedWheels.length === 0) return;

  const mass = Math.max(0.5, chassis.mass());
  const drivenCount = Math.max(1, plantedWheels.filter((w) => w.def.driven).length);
  // Budget from maxForce / maxAccel — hub friction is low; we own grip.
  const perCap = (Math.max(drive.maxAccel, 1) * mass * dt) / drivenCount;
  const forceCap = (drive.maxForce * dt) / drivenCount;
  const motorCap = Math.min(perCap * 2.5, forceCap);

  for (const wheel of plantedWheels) {
    if (!wheel.def.driven && !(wheel.def.steered && Math.abs(state.steer) > 0.02)) continue;

    const hub = hubWorldPosition(wheel.axle, wheel.def.hubOffset);
    const steer = wheel.def.steered ? state.steer * drive.steerAngle : 0;
    forward.set(0, 0, -1).applyQuaternion(q);
    if (steer !== 0) forward.applyAxisAngle(up, steer);
    const into = forward.dot(up);
    forward.addScaledVector(up, -into);
    if (forward.lengthSq() < 1e-6) continue;
    forward.normalize();

    // Chassis velocity at hub — drive the vehicle unit, not the free axle.
    const vel = chassis.velocityAtPoint({ x: hub.x, y: hub.y, z: hub.z });
    const vLong = vel.x * forward.x + vel.y * forward.y + vel.z * forward.z;

    if (wheel.def.driven) {
      const err = state.commandSpeed - vLong;
      const j = clamp(err * mass * 0.9 * dt, -motorCap, motorCap);
      // COM impulse — does not pitch-fight soft links; hubs are low-friction plant only.
      chassis.applyImpulse({ x: forward.x * j, y: 0, z: forward.z * j }, true);
    }

    // Front lateral at hub while steering → yaw without axle shear stack.
    if (wheel.def.steered && Math.abs(state.steer) > 0.02) {
      side.crossVectors(up, forward);
      if (side.lengthSq() > 1e-6) {
        side.normalize();
        const vLat = vel.x * side.x + vel.y * side.y + vel.z * side.z;
        const targetLat = state.steer * Math.abs(state.commandSpeed) * 0.35;
        const jLat = clamp((targetLat - vLat) * mass * 0.45 * dt, -motorCap * 1.2, motorCap * 1.2);
        chassis.applyImpulseAtPoint(
          { x: side.x * jLat, y: 0, z: side.z * jLat },
          { x: hub.x, y: hub.y, z: hub.z },
          true
        );
      }
    }
  }

  // Soft sphericals otherwise soak COM thrust — keep axle planar vel with chassis while driving.
  const cl = chassis.linvel();
  for (const axle of axles.values()) {
    const al = axle.body.linvel();
    axle.body.setLinvel({ x: cl.x, y: al.y, z: cl.z }, true);
  }

  const lin = chassis.linvel();
  const planar = Math.hypot(lin.x, lin.z);
  if (planar > rig.maxSpeed * 1.2) {
    const s = (rig.maxSpeed * 1.2) / planar;
    chassis.setLinvel({ x: lin.x * s, y: lin.y, z: lin.z * s }, true);
    for (const axle of axles.values()) {
      const al = axle.body.linvel();
      axle.body.setLinvel({ x: lin.x * s, y: al.y, z: lin.z * s }, true);
    }
  }
}
