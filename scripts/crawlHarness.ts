/**
 * Shared headless harness for crawl tests (no browser).
 * PhysicsWorld + TrailScene + CrawlerVehicle, fixed dt, scripted input.
 */
import * as THREE from "three";
import { loadStockRig } from "../src/data/loadRig.ts";
import { PhysicsWorld } from "../src/physics/PhysicsWorld.ts";
import { CrawlerVehicle } from "../src/vehicles/CrawlerVehicle.ts";
import { TrailScene } from "../src/world/TrailScene.ts";
import type { DriveInput } from "../src/input/Input.ts";
import type { RigDef } from "../src/vehicles/types.ts";

export const DT = 1 / 60;

export type Harness = {
  physics: PhysicsWorld;
  trail: TrailScene;
  vehicle: CrawlerVehicle;
  rig: RigDef;
};

export async function createHarness(): Promise<Harness> {
  await PhysicsWorld.init();
  const rig = loadStockRig();
  const physics = PhysicsWorld.create();
  physics.world.integrationParameters.numSolverIterations = 28;
  physics.world.integrationParameters.normalizedAllowedLinearError = 0.0005;
  const trail = new TrailScene(physics, rig.spawn);
  const vehicle = new CrawlerVehicle(physics, trail.scene, rig);
  return { physics, trail, vehicle, rig };
}

export function uprightY(vehicle: CrawlerVehicle): number {
  const rot = vehicle.chassisBody.rotation();
  const q = new THREE.Quaternion(rot.x, rot.y, rot.z, rot.w);
  return new THREE.Vector3(0, 1, 0).applyQuaternion(q).y;
}

export function speed(vehicle: CrawlerVehicle): number {
  const v = vehicle.chassisBody.linvel();
  return Math.hypot(v.x, v.y, v.z);
}

export function planarSpeed(vehicle: CrawlerVehicle): number {
  const v = vehicle.chassisBody.linvel();
  return Math.hypot(v.x, v.z);
}

export function yawY(vehicle: CrawlerVehicle): number {
  const rot = vehicle.chassisBody.rotation();
  return new THREE.Euler().setFromQuaternion(
    new THREE.Quaternion(rot.x, rot.y, rot.z, rot.w),
    "YXZ"
  ).y;
}

/** Chassis-local forward (0,0,-1) in world XZ. */
export function forwardXZ(vehicle: CrawlerVehicle): { x: number; z: number } {
  const rot = vehicle.chassisBody.rotation();
  const q = new THREE.Quaternion(rot.x, rot.y, rot.z, rot.w);
  const f = new THREE.Vector3(0, 0, -1).applyQuaternion(q);
  const len = Math.hypot(f.x, f.z) || 1;
  return { x: f.x / len, z: f.z / len };
}

export function step(h: Harness, input: DriveInput, dt = DT): void {
  h.vehicle.preStep(h.physics.world, input, dt);
  h.physics.step(dt);
  h.vehicle.syncMeshes();
}

export function idle(h: Harness, frames: number, dt = DT): void {
  for (let i = 0; i < frames; i += 1) {
    step(h, { throttle: 0, steer: 0, reset: false }, dt);
  }
}

export function drive(
  h: Harness,
  frames: number,
  throttle: number,
  steer = 0,
  dt = DT
): { maxSpeed: number; minUpright: number } {
  let maxSpeed = 0;
  let minUpright = 1;
  for (let i = 0; i < frames; i += 1) {
    step(h, { throttle, steer, reset: false }, dt);
    maxSpeed = Math.max(maxSpeed, speed(h.vehicle));
    minUpright = Math.min(minUpright, uprightY(h.vehicle));
  }
  return { maxSpeed, minUpright };
}

/** Full vehicle reset then optional relocate (clears hub drive state). */
export function place(
  h: Harness,
  x: number,
  y: number,
  z: number,
  yaw = 0
): void {
  h.vehicle.reset();
  const q = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, yaw, 0, "YXZ"));
  h.vehicle.chassisBody.setTranslation({ x, y, z }, true);
  h.vehicle.chassisBody.setRotation({ x: q.x, y: q.y, z: q.z, w: q.w }, true);
  h.vehicle.chassisBody.setLinvel({ x: 0, y: 0, z: 0 }, true);
  h.vehicle.chassisBody.setAngvel({ x: 0, y: 0, z: 0 }, true);
  h.vehicle.kitSuspension()?.reset(h.vehicle.chassisBody);
}

export type GateFailure = { gate: string; detail: string };

export function check(
  failures: GateFailure[],
  gate: string,
  ok: boolean,
  detail: string
): void {
  if (!ok) failures.push({ gate, detail });
}
