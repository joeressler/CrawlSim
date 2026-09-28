/**
 * Soft locate assist for Strategy B kit: wheelbase consistency, axle yaw limits,
 * and skid-routed drivetrain stiffener (transfer + F/R shafts).
 *
 * Impulse / soft-spring only — NO Rapier lock/fixed joints. Hard joints fight the
 * spherical 4-link + panhard and crumple the hang. Links remain the primary locate;
 * this only damps inchworm WB break and runaway axle yaw.
 *
 * Hard gates preserved elsewhere: one vertical plant (hub spheres), coilover
 * restLength = mounts, tire meshes on hubs.
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
const scratchX = new THREE.Vector3();
const lookMat = new THREE.Matrix4();

/** Soft WB spring between F/R axle centers (N/m, N·s/m). Mild — links still own locate. */
const WB_K = 70;
const WB_C = 6;
const WB_FORCE_CAP = 9;

/** Soft driveshaft length springs: chassis transfer → axle pumpkin. */
const SHAFT_K = 55;
const SHAFT_C = 5;
const SHAFT_FORCE_CAP = 6;

/** Axle yaw vs chassis: soft restore past SOFT, strong past HARD (rad). */
const YAW_SOFT = 0.14;
const YAW_HARD = 0.5;
const YAW_K = 3.2;
const YAW_C = 0.55;
const YAW_TORQUE_CAP = 0.35;

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

function clamp(v: number, lo: number, hi: number): number {
  if (!Number.isFinite(v)) return 0;
  return Math.min(hi, Math.max(lo, v));
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
  // Geometry is Z-aligned after rotateX; scale length along Z.
  mesh.scale.set(1, 1, dist / Math.max(restLength, 0.02));
}

/**
 * Signed yaw of axle tube (local +X) relative to chassis, about chassis up.
 * 0 at rest BOM pose. Positive = axle spun CCW looking down chassis up.
 */
function axleYawAboutChassisUp(chassis: RAPIER.RigidBody, axle: RAPIER.RigidBody): number {
  const cr = chassis.rotation();
  const ar = axle.rotation();
  scratchQ.set(cr.x, cr.y, cr.z, cr.w);
  scratchUp.set(0, 1, 0).applyQuaternion(scratchQ);
  scratchX.set(1, 0, 0).applyQuaternion(scratchQ);
  const aq = new THREE.Quaternion(ar.x, ar.y, ar.z, ar.w);
  scratchA.set(1, 0, 0).applyQuaternion(aq);
  const alongUp = scratchA.dot(scratchUp);
  scratchA.addScaledVector(scratchUp, -alongUp);
  if (scratchA.lengthSq() < 1e-8) return 0;
  scratchA.normalize();
  const latDot = scratchX.dot(scratchUp);
  scratchX.addScaledVector(scratchUp, -latDot);
  if (scratchX.lengthSq() < 1e-8) return 0;
  scratchX.normalize();
  const cos = clamp(scratchA.dot(scratchX), -1, 1);
  scratchB.crossVectors(scratchX, scratchA);
  const sin = scratchUp.dot(scratchB);
  return Math.atan2(sin, cos);
}

function applySoftDistance(
  a: RAPIER.RigidBody,
  b: RAPIER.RigidBody,
  pa: { x: number; y: number; z: number },
  pb: { x: number; y: number; z: number },
  restLength: number,
  springK: number,
  damperC: number,
  forceCap: number,
  dt: number
): void {
  const dx = pb.x - pa.x;
  const dy = pb.y - pa.y;
  const dz = pb.z - pa.z;
  const dist = Math.hypot(dx, dy, dz);
  if (dist < 1e-5 || !Number.isFinite(dist)) return;
  const ux = dx / dist;
  const uy = dy / dist;
  const uz = dz / dist;
  const err = dist - restLength;

  const va = a.velocityAtPoint(pa);
  const vb = b.velocityAtPoint(pb);
  const closing = (vb.x - va.x) * ux + (vb.y - va.y) * uy + (vb.z - va.z) * uz;

  let force = springK * err + damperC * closing;
  force = clamp(force, -forceCap, forceCap);
  const impulse = force * dt;
  a.applyImpulseAtPoint({ x: ux * impulse, y: uy * impulse, z: uz * impulse }, pa, true);
  b.applyImpulseAtPoint({ x: -ux * impulse, y: -uy * impulse, z: -uz * impulse }, pb, true);
}

function applyYawLimit(chassis: RAPIER.RigidBody, axle: RAPIER.RigidBody, dt: number): void {
  const yaw = axleYawAboutChassisUp(chassis, axle);
  const abs = Math.abs(yaw);
  if (abs < YAW_SOFT) return;

  const cr = chassis.rotation();
  scratchQ.set(cr.x, cr.y, cr.z, cr.w);
  scratchUp.set(0, 1, 0).applyQuaternion(scratchQ);

  // Excess past soft band; ramp hard near YAW_HARD
  const excess = abs - YAW_SOFT;
  const hardSpan = Math.max(1e-3, YAW_HARD - YAW_SOFT);
  const hardBlend = clamp(excess / hardSpan, 0, 1);
  const stiffness = YAW_K * (1 + 4 * hardBlend * hardBlend);

  const ang = axle.angvel();
  const omega = ang.x * scratchUp.x + ang.y * scratchUp.y + ang.z * scratchUp.z;
  const sign = yaw >= 0 ? 1 : -1;
  let torque = -sign * stiffness * excess - YAW_C * omega;
  torque = clamp(torque, -YAW_TORQUE_CAP, YAW_TORQUE_CAP);
  const j = torque * dt;
  axle.applyTorqueImpulse(
    { x: scratchUp.x * j, y: scratchUp.y * j, z: scratchUp.z * j },
    true
  );

  // Near hard limit: bleed axle yaw rate so it cannot 180° spin through.
  if (abs > YAW_HARD * 0.85) {
    const bleed = 0.65;
    axle.setAngvel(
      {
        x: ang.x - scratchUp.x * omega * bleed,
        y: ang.y - scratchUp.y * omega * bleed,
        z: ang.z - scratchUp.z * omega * bleed,
      },
      true
    );
  }
}

function buildShaftMesh(restLength: number, name: string): THREE.Mesh {
  const mesh = new THREE.Mesh(
    new THREE.CylinderGeometry(SHAFT_RADIUS, SHAFT_RADIUS, Math.max(restLength, 0.05), 8),
    shaftMat
  );
  mesh.name = name;
  // Default cylinder is Y-up; meshLookRotation builds Z-forward basis — rotate so Y→Z.
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

  // Transfer case rides on chassis visual (skid-mounted).
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

  // Initial shaft pose
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

/**
 * Soft WB + skid shaft stiffeners + axle yaw limits.
 * Safe no-op if front/rear missing. Never throws on per-frame math.
 */
export function applyKitLocate(
  chassis: RAPIER.RigidBody,
  axles: Map<string, KitAxleRuntime>,
  locate: KitLocateRuntime,
  dt: number
): void {
  if (!(dt > 0)) return;
  const front = axles.get("front");
  const rear = axles.get("rear");
  if (!front || !rear) return;

  const ft = front.body.translation();
  const rt = rear.body.translation();
  const frontCenter = { x: ft.x, y: ft.y, z: ft.z };
  const rearCenter = { x: rt.x, y: rt.y, z: rt.z };

  const cr = chassis.rotation();
  scratchQ.set(cr.x, cr.y, cr.z, cr.w);
  // Chassis lateral (+X) and forward (-Z) unit vectors.
  scratchX.set(1, 0, 0).applyQuaternion(scratchQ);
  const latLen = scratchX.length();
  if (latLen > 1e-6) scratchX.multiplyScalar(1 / latLen);
  scratchA.set(0, 0, -1).applyQuaternion(scratchQ);
  const fLen = scratchA.length();
  if (fLen > 1e-6) scratchA.multiplyScalar(1 / fLen);

  // 1) Soft wheelbase spring between axle centers (anti-inchworm; internal to axles).
  applySoftDistance(
    front.body,
    rear.body,
    frontCenter,
    rearCenter,
    locate.restWheelbase,
    WB_K,
    WB_C,
    WB_FORCE_CAP,
    dt
  );

  // 2) Skid drivetrain: lateral-only stiffener (yaw), not length spring (length soaked drive).
  //    Transfer is on chassis centerline; pumpkin should stay near x=0 in chassis frame.
  const transfer = worldPoint(chassis, locate.transferLocal);
  applyLateralShaft(
    chassis,
    front.body,
    transfer,
    frontCenter,
    scratchX.x,
    scratchX.y,
    scratchX.z,
    dt
  );
  applyLateralShaft(
    chassis,
    rear.body,
    transfer,
    rearCenter,
    scratchX.x,
    scratchX.y,
    scratchX.z,
    dt
  );

  // 3) Axle yaw soft limits (prevent 180? spin).
  applyYawLimit(chassis, front.body, dt);
  applyYawLimit(chassis, rear.body, dt);
}

/** Soft spring on chassis-lateral separation between transfer and axle pumpkin. */
function applyLateralShaft(
  chassis: RAPIER.RigidBody,
  axle: RAPIER.RigidBody,
  transfer: { x: number; y: number; z: number },
  pumpkin: { x: number; y: number; z: number },
  lx: number,
  ly: number,
  lz: number,
  dt: number
): void {
  const dx = pumpkin.x - transfer.x;
  const dy = pumpkin.y - transfer.y;
  const dz = pumpkin.z - transfer.z;
  const latErr = dx * lx + dy * ly + dz * lz; // pumpkin lateral vs transfer (rest ~0)
  const va = chassis.velocityAtPoint(transfer);
  const vb = axle.velocityAtPoint(pumpkin);
  const latClosing = (vb.x - va.x) * lx + (vb.y - va.y) * ly + (vb.z - va.z) * lz;
  let force = SHAFT_K * latErr + SHAFT_C * latClosing;
  force = clamp(force, -SHAFT_FORCE_CAP, SHAFT_FORCE_CAP);
  const impulse = force * dt;
  // Positive latErr = pumpkin too far +X ? pull pumpkin -X / push chassis +X at transfer
  chassis.applyImpulseAtPoint({ x: lx * impulse, y: ly * impulse, z: lz * impulse }, transfer, true);
  axle.applyImpulseAtPoint({ x: -lx * impulse, y: -ly * impulse, z: -lz * impulse }, pumpkin, true);
}
