/**
 * Soft locate assists are OFF the kit hot path (DOF ownership: spherical joints locate).
 * This module keeps transfer/driveshaft visuals only. applyKitLocate / applyHangCeiling
 * remain as no-ops so older call sites compile; do not re-enable soft WB / yaw snaps /
 * hang ceiling without revisiting constraint fighting.
 *
 * SHAFT_ENABLED stays false — panhard owns lateral; shaft springs soak drive.
 */
import type RAPIER from "@dimforge/rapier3d-compat";
import * as THREE from "three";
import type { KitDef, Vec3 } from "./types.ts";
import type { KitAxleRuntime } from "./kitBodies.ts";

export type KitLocateRuntime = {
  restWheelbase: number;
  transferLocal: Vec3;
  frontShaftRest: number;
  rearShaftRest: number;
  transferMesh: THREE.Object3D;
  frontShaftMesh: THREE.Object3D;
  rearShaftMesh: THREE.Object3D;
  syncMeshes: (chassis: RAPIER.RigidBody, axles: Map<string, KitAxleRuntime>) => void;
};

const scratchQ = new THREE.Quaternion();
const scratchV = new THREE.Vector3();
const scratchA = new THREE.Vector3();
const scratchB = new THREE.Vector3();
const scratchUp = new THREE.Vector3();
const lookMat = new THREE.Matrix4();

/** Soft driveshaft lateral — must stay false (panhard owns lateral locate). */
const SHAFT_ENABLED = false;
void SHAFT_ENABLED;

const TRANSFER_LOCAL: Vec3 = { x: 0, y: -0.034, z: 0 };
const SHAFT_RADIUS = 0.0045;
const TRANSFER_HALF = { x: 0.018, y: 0.012, z: 0.028 };

const shaftMat = new THREE.MeshStandardMaterial({
  color: 0x8a9098,
  metalness: 0.75,
  roughness: 0.28,
});
const transferMat = new THREE.MeshStandardMaterial({
  color: 0x2a2e34,
  metalness: 0.45,
  roughness: 0.5,
});

function worldPoint(body: RAPIER.RigidBody, local: Vec3): { x: number; y: number; z: number } {
  const t = body.translation();
  const r = body.rotation();
  scratchQ.set(r.x, r.y, r.z, r.w);
  scratchV.set(local.x, local.y, local.z).applyQuaternion(scratchQ);
  return { x: t.x + scratchV.x, y: t.y + scratchV.y, z: t.z + scratchV.z };
}

/** Orient mesh so local +Z points from->to (cylinder geometry is pre-rotated Y->Z). */
function quatLookZ(fx: number, fy: number, fz: number, tx: number, ty: number, tz: number): THREE.Quaternion {
  const dx = tx - fx;
  const dy = ty - fy;
  const dz = tz - fz;
  const length = Math.hypot(dx, dy, dz);
  if (length < 1e-6) return new THREE.Quaternion();
  scratchA.set(dx / length, dy / length, dz / length);
  const useXUp = Math.abs(scratchA.y) > 0.92;
  scratchUp.set(useXUp ? 1 : 0, useXUp ? 0 : 1, 0);
  scratchB.crossVectors(scratchUp, scratchA);
  if (scratchB.lengthSq() < 1e-8) scratchB.set(1, 0, 0);
  scratchB.normalize();
  scratchUp.crossVectors(scratchA, scratchB).normalize();
  lookMat.makeBasis(scratchB, scratchUp, scratchA);
  return new THREE.Quaternion().setFromRotationMatrix(lookMat);
}

function syncShaftMesh(
  mesh: THREE.Object3D,
  from: { x: number; y: number; z: number },
  to: { x: number; y: number; z: number },
  restLength: number
): void {
  const fx = from.x;
  const fy = from.y;
  const fz = from.z;
  const tx = to.x;
  const ty = to.y;
  const tz = to.z;
  const dist = Math.max(Math.hypot(tx - fx, ty - fy, tz - fz), 0.02);
  mesh.position.set((fx + tx) * 0.5, (fy + ty) * 0.5, (fz + tz) * 0.5);
  mesh.quaternion.copy(quatLookZ(fx, fy, fz, tx, ty, tz));
  mesh.scale.set(1, 1, dist / Math.max(restLength, 0.02));
}

function buildShaftMesh(restLength: number, name: string): THREE.Mesh {
  const mesh = new THREE.Mesh(
    new THREE.CylinderGeometry(SHAFT_RADIUS, SHAFT_RADIUS, Math.max(restLength, 0.05), 8),
    shaftMat
  );
  mesh.name = name;
  mesh.geometry.rotateX(Math.PI / 2);
  return mesh;
}

export function buildKitLocate(
  scene: THREE.Scene,
  chassisMesh: THREE.Object3D,
  chassis: RAPIER.RigidBody,
  _kit: KitDef,
  axles: Map<string, KitAxleRuntime>
): KitLocateRuntime | null {
  const front = axles.get("front");
  const rear = axles.get("rear");
  if (!front || !rear) return null;

  const restWheelbase = Math.max(
    0.2,
    Math.hypot(
      front.restLocal.x - rear.restLocal.x,
      front.restLocal.y - rear.restLocal.y,
      front.restLocal.z - rear.restLocal.z
    )
  );

  const transferLocal: Vec3 = { ...TRANSFER_LOCAL };
  const transferWorld = worldPoint(chassis, transferLocal);
  const frontPumpkin = worldPoint(front.body, { x: 0, y: 0, z: 0 });
  const rearPumpkin = worldPoint(rear.body, { x: 0, y: 0, z: 0 });
  const frontShaftRest = Math.max(
    0.05,
    Math.hypot(
      frontPumpkin.x - transferWorld.x,
      frontPumpkin.y - transferWorld.y,
      frontPumpkin.z - transferWorld.z
    )
  );
  const rearShaftRest = Math.max(
    0.05,
    Math.hypot(
      rearPumpkin.x - transferWorld.x,
      rearPumpkin.y - transferWorld.y,
      rearPumpkin.z - transferWorld.z
    )
  );

  const transferMesh = new THREE.Group();
  transferMesh.name = "transfer_case";
  transferMesh.position.set(transferLocal.x, transferLocal.y, transferLocal.z);
  const caseBox = new THREE.Mesh(
    new THREE.BoxGeometry(TRANSFER_HALF.x * 2, TRANSFER_HALF.y * 2, TRANSFER_HALF.z * 2),
    transferMat
  );
  transferMesh.add(caseBox);
  const yoke = new THREE.Mesh(
    new THREE.CylinderGeometry(0.006, 0.006, 0.02, 8),
    shaftMat
  );
  yoke.rotation.z = Math.PI / 2;
  yoke.position.set(0, -0.002, 0);
  transferMesh.add(yoke);
  chassisMesh.add(transferMesh);

  const frontShaftMesh = buildShaftMesh(frontShaftRest, "front_driveshaft");
  const rearShaftMesh = buildShaftMesh(rearShaftRest, "rear_driveshaft");
  scene.add(frontShaftMesh);
  scene.add(rearShaftMesh);

  const syncMeshes = (chassisBody: RAPIER.RigidBody, axleMap: Map<string, KitAxleRuntime>): void => {
    const f = axleMap.get("front");
    const r = axleMap.get("rear");
    if (!f || !r) return;
    const transfer = worldPoint(chassisBody, transferLocal);
    const fp = worldPoint(f.body, { x: 0, y: 0, z: 0 });
    const rp = worldPoint(r.body, { x: 0, y: 0, z: 0 });
    syncShaftMesh(frontShaftMesh, transfer, fp, frontShaftRest);
    syncShaftMesh(rearShaftMesh, transfer, rp, rearShaftRest);
  };

  syncMeshes(chassis, axles);

  return {
    restWheelbase,
    transferLocal,
    frontShaftRest,
    rearShaftRest,
    transferMesh,
    frontShaftMesh,
    rearShaftMesh,
    syncMeshes,
  };
}

/** Hot-path soft locate removed — spherical joints locate axles. */
export function applyKitLocate(
  _chassis: RAPIER.RigidBody,
  _axles: Map<string, KitAxleRuntime>,
  _locate: KitLocateRuntime,
  _dt: number
): void {
  // no-op
}

/** Hang ceiling removed — coilovers + crumple pads own ride/fold stop. */
export function applyHangCeiling(
  _chassis: RAPIER.RigidBody,
  _axle: KitAxleRuntime,
  _dt: number
): void {
  // no-op
}
