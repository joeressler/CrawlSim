/**
 * Headless idle settle gate for Strategy B Phase 2a.
 * Measures late-window activity (last 1.0s) so early ray/joint settle is ignored.
 */
import * as THREE from "three";
import { loadStockRig } from "../src/data/loadRig.ts";
import { PhysicsWorld } from "../src/physics/PhysicsWorld.ts";
import { CrawlerVehicle } from "../src/vehicles/CrawlerVehicle.ts";
import { TrailScene } from "../src/world/TrailScene.ts";

export type SettleMetrics = {
  maxAbsVy: number;
  yawRad: number;
  planarDrift: number;
  axleMaxAbsVy: number;
  yawRate: number;
  planarRate: number;
  frames: number;
  dt: number;
  seconds: number;
};

export async function runIdleSettle(seconds = 4, dt = 1 / 60): Promise<SettleMetrics> {
  await PhysicsWorld.init();
  const rig = loadStockRig();
  const physics = PhysicsWorld.create();
  physics.world.integrationParameters.numSolverIterations = 14;
  const trail = new TrailScene(physics, rig.spawn);
  const vehicle = new CrawlerVehicle(physics, trail.scene, rig);

  const frames = Math.round(seconds / dt);
  const windowSec = 1.0;
  const windowFrames = Math.round(windowSec / dt);
  const measureAfter = frames - windowFrames;

  let maxAbsVy = 0;
  let axleMaxAbsVy = 0;
  let yawStart = 0;
  let planarStart = { x: 0, z: 0 };
  let marked = false;

  const origin = vehicle.chassisBody.translation();

  for (let i = 0; i < frames; i += 1) {
    vehicle.preStep(physics.world, { throttle: 0, steer: 0, reset: false }, dt);
    physics.step(dt);
    vehicle.syncMeshes();

    if (i === measureAfter) {
      const t = vehicle.chassisBody.translation();
      const rot = vehicle.chassisBody.rotation();
      yawStart = new THREE.Euler().setFromQuaternion(
        new THREE.Quaternion(rot.x, rot.y, rot.z, rot.w),
        "YXZ"
      ).y;
      planarStart = { x: t.x, z: t.z };
      marked = true;
      maxAbsVy = 0;
      axleMaxAbsVy = 0;
    }
    if (i < measureAfter) continue;

    maxAbsVy = Math.max(maxAbsVy, Math.abs(vehicle.chassisBody.linvel().y));
    const kit = vehicle.kitSuspension();
    if (kit) {
      for (const axle of kit.axles.values()) {
        axleMaxAbsVy = Math.max(axleMaxAbsVy, Math.abs(axle.body.linvel().y));
      }
    }
  }

  const end = vehicle.chassisBody.translation();
  const rot = vehicle.chassisBody.rotation();
  const yawEnd = new THREE.Euler().setFromQuaternion(
    new THREE.Quaternion(rot.x, rot.y, rot.z, rot.w),
    "YXZ"
  ).y;

  const yawRad = marked ? yawEnd - yawStart : yawEnd;
  const planarDrift = marked
    ? Math.hypot(end.x - planarStart.x, end.z - planarStart.z)
    : Math.hypot(end.x - origin.x, end.z - origin.z);

  return {
    maxAbsVy,
    yawRad,
    planarDrift,
    axleMaxAbsVy,
    yawRate: yawRad / windowSec,
    planarRate: planarDrift / windowSec,
    frames,
    dt,
    seconds,
  };
}

/** Late-window idle calm (per second). */
const MAX_ABS_VY = 0.35;
const MAX_AXLE_ABS_VY = 0.45;
const MAX_YAW = 0.06;
const MAX_PLANAR = 0.08;

const metrics = await runIdleSettle();
const ok =
  metrics.maxAbsVy <= MAX_ABS_VY &&
  metrics.axleMaxAbsVy <= MAX_AXLE_ABS_VY &&
  Math.abs(metrics.yawRad) <= MAX_YAW &&
  metrics.planarDrift <= MAX_PLANAR;

console.log(JSON.stringify({ ok, thresholds: { MAX_ABS_VY, MAX_AXLE_ABS_VY, MAX_YAW, MAX_PLANAR }, metrics }, null, 2));
if (!ok) {
  console.error("idle settle gate FAILED");
  process.exit(1);
}
console.log("idle settle gate OK");