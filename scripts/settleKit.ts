import { fileURLToPath } from "node:url";
import * as THREE from "three";
import { loadStockRig } from "../src/data/loadRig.ts";
import { PhysicsWorld } from "../src/physics/PhysicsWorld.ts";
import { CrawlerVehicle } from "../src/vehicles/CrawlerVehicle.ts";
import { TrailScene } from "../src/world/TrailScene.ts";

/**
 * Idle settle metrics (importable). CLI runs only when this file is the entrypoint.
 */
export type SettleMetrics = {
  maxAbsVy: number;
  yawRad: number;
  planarDrift: number;
  axleMaxAbsVy: number;
  idleGap: number;
  chassisY: number;
  upright: number;
  relHang: number;
  driveDeltaZ: number;
  steerYaw: number;
  reverseSteerYaw: number;
  frames: number;
  dt: number;
  seconds: number;
};

export async function runIdleSettle(seconds = 5, dt = 1 / 60): Promise<SettleMetrics> {
  await PhysicsWorld.init();
  const rig = loadStockRig();
  const physics = PhysicsWorld.create();
  physics.world.integrationParameters.numSolverIterations = 28;
  physics.world.integrationParameters.normalizedAllowedLinearError = 0.0005;
  const trail = new TrailScene(physics, rig.spawn);
  const vehicle = new CrawlerVehicle(physics, trail.scene, rig);

  const frames = Math.round(seconds / dt);
  const windowSec = 1.2;
  const windowFrames = Math.round(windowSec / dt);
  const measureAfter = frames - windowFrames;

  let maxAbsVy = 0;
  let axleMaxAbsVy = 0;
  let yawStart = 0;
  let planarStart = { x: 0, z: 0 };
  let marked = false;
  let idleGap = 0;
  let upright = 1;
  let relHang = 0;

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
      const ch = vehicle.chassisBody.translation();
      const ax = kit.axles.get("front")!.body.translation();
      relHang = ch.y - ax.y;
    }
    idleGap = vehicle.minHubClearance();
    const rot = vehicle.chassisBody.rotation();
    const q = new THREE.Quaternion(rot.x, rot.y, rot.z, rot.w);
    const up = new THREE.Vector3(0, 1, 0).applyQuaternion(q);
    upright = up.y;
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
    : 0;

  // Drive smoke: throttle forward.
  // Steer / reverse BEFORE sustained W drive. Drive-armed hang during W warm-starts
  // contacts and starves post-reset steer yaw (false settle fail).
  vehicle.reset();
  for (let i = 0; i < 30; i += 1) {
    vehicle.preStep(physics.world, { throttle: 0, steer: 0, reset: false }, dt);
    physics.step(dt);
  }
  const yawBefore = new THREE.Euler().setFromQuaternion(
    new THREE.Quaternion(
      vehicle.chassisBody.rotation().x,
      vehicle.chassisBody.rotation().y,
      vehicle.chassisBody.rotation().z,
      vehicle.chassisBody.rotation().w
    ),
    "YXZ"
  ).y;
  for (let i = 0; i < 140; i += 1) {
    vehicle.preStep(physics.world, { throttle: 0.7, steer: 1, reset: false }, dt);
    physics.step(dt);
    vehicle.syncMeshes();
  }
  const yawAfter = new THREE.Euler().setFromQuaternion(
    new THREE.Quaternion(
      vehicle.chassisBody.rotation().x,
      vehicle.chassisBody.rotation().y,
      vehicle.chassisBody.rotation().z,
      vehicle.chassisBody.rotation().w
    ),
    "YXZ"
  ).y;
  const steerYaw = yawAfter - yawBefore;

  vehicle.reset();
  for (let i = 0; i < 40; i += 1) {
    vehicle.preStep(physics.world, { throttle: 0, steer: 0, reset: false }, dt);
    physics.step(dt);
  }
  for (let i = 0; i < 90; i += 1) {
    vehicle.preStep(physics.world, { throttle: -1, steer: 0, reset: false }, dt);
    physics.step(dt);
    vehicle.syncMeshes();
  }
  const revYawBefore = new THREE.Euler().setFromQuaternion(
    new THREE.Quaternion(
      vehicle.chassisBody.rotation().x,
      vehicle.chassisBody.rotation().y,
      vehicle.chassisBody.rotation().z,
      vehicle.chassisBody.rotation().w
    ),
    "YXZ"
  ).y;
  for (let i = 0; i < 100; i += 1) {
    vehicle.preStep(physics.world, { throttle: -0.8, steer: 1, reset: false }, dt);
    physics.step(dt);
    vehicle.syncMeshes();
  }
  const revYawAfter = new THREE.Euler().setFromQuaternion(
    new THREE.Quaternion(
      vehicle.chassisBody.rotation().x,
      vehicle.chassisBody.rotation().y,
      vehicle.chassisBody.rotation().z,
      vehicle.chassisBody.rotation().w
    ),
    "YXZ"
  ).y;
  const reverseSteerYaw = revYawAfter - revYawBefore;

  // Drive smoke last (hang may arm).
  vehicle.reset();
  for (let i = 0; i < 40; i += 1) {
    vehicle.preStep(physics.world, { throttle: 0, steer: 0, reset: false }, dt);
    physics.step(dt);
  }
  const z0 = vehicle.chassisBody.translation().z;
  for (let i = 0; i < 200; i += 1) {
    vehicle.preStep(physics.world, { throttle: 1, steer: 0, reset: false }, dt);
    physics.step(dt);
    vehicle.syncMeshes();
  }
  const driveDeltaZ = z0 - vehicle.chassisBody.translation().z;

  return {
    maxAbsVy,
    yawRad,
    planarDrift,
    axleMaxAbsVy,
    idleGap,
    chassisY: end.y,
    upright,
    relHang,
    driveDeltaZ,
    steerYaw,
    reverseSteerYaw,
    frames,
    dt,
    seconds,
  };
}



const isSettleCli = process.argv[1] !== undefined && fileURLToPath(import.meta.url) === process.argv[1];

if (isSettleCli) {
  const MAX_ABS_VY = 0.35;
  const MAX_AXLE_ABS_VY = 0.5;
  const MAX_YAW = 0.25;
  const MAX_PLANAR = 0.2;
  const MIN_IDLE_GAP = -0.02;
  const MAX_IDLE_GAP = 0.04;
  const MIN_DRIVE_DZ = 0.08;
  const MIN_CHASSIS_Y = 0.07;
  const MIN_UPRIGHT = 0.75;
  const MIN_REL_HANG = 0.03; // chassis above axle â€” no crumple
  const MIN_STEER_YAW = 0.05;
  const MIN_REVERSE_STEER_YAW = 0.04;

  const metrics = await runIdleSettle();
  const ok =
    metrics.maxAbsVy <= MAX_ABS_VY &&
    metrics.axleMaxAbsVy <= MAX_AXLE_ABS_VY &&
    Math.abs(metrics.yawRad) <= MAX_YAW &&
    metrics.planarDrift <= MAX_PLANAR &&
    metrics.idleGap >= MIN_IDLE_GAP &&
    metrics.idleGap <= MAX_IDLE_GAP &&
    metrics.driveDeltaZ >= MIN_DRIVE_DZ &&
    metrics.chassisY >= MIN_CHASSIS_Y &&
    metrics.upright >= MIN_UPRIGHT &&
    metrics.relHang >= MIN_REL_HANG &&
    Math.abs(metrics.steerYaw) >= MIN_STEER_YAW &&
    Math.abs(metrics.reverseSteerYaw) >= MIN_REVERSE_STEER_YAW;

  console.log(
    JSON.stringify(
      {
        ok,
        thresholds: {
          MAX_ABS_VY,
          MAX_AXLE_ABS_VY,
          MAX_YAW,
          MAX_PLANAR,
          MIN_IDLE_GAP,
          MAX_IDLE_GAP,
          MIN_DRIVE_DZ,
          MIN_CHASSIS_Y,
          MIN_UPRIGHT,
          MIN_REL_HANG,
          MIN_STEER_YAW,
          MIN_REVERSE_STEER_YAW,
        },
        metrics,
      },
      null,
      2
    )
  );
  if (!ok) {
    console.error("idle settle gate FAILED");
    process.exit(1);
  }
  console.log("idle settle gate OK");
}

