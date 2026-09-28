/**
 * Strategy B Phase 3: dynamic axle bodies + spherical link kit + soft hub plant spheres.
 * Gravity restored on kit parts. Vertical plant = hub spheres only (no chassis-ray spring).
 * Coilovers / hub drive live in kitCoilovers.ts + kitHubDrive.ts.
 */
import RAPIER from "@dimforge/rapier3d-compat";
import * as THREE from "three";
import { CHASSIS_GROUPS, HUB_GROUPS, KIT_PART_GROUPS } from "../physics/collisionGroups.ts";
import { syncRigidBodyToObject } from "../physics/sync.ts";
import { buildScxAxleVisual, buildScxLinkVisual } from "./scxVisuals.ts";
import type { AxleDef, KitDef, KitWheelDef, LinkDef, MountDef, MountRef, Vec3 } from "./types.ts";

const AXLE_HALF = { x: 0.105, y: 0.016, z: 0.016 };
const AXLE_MASS = 0.4;
const LINK_MASS = 0.05;
const LINK_RADIUS = 0.01;

export type KitAxleRuntime = {
  id: string;
  body: RAPIER.RigidBody;
  mesh: THREE.Object3D;
  def: AxleDef;
  restLocal: Vec3;
};

export type KitLinkRuntime = {
  id: string;
  body: RAPIER.RigidBody;
  mesh: THREE.Object3D;
  def: LinkDef;
  length: number;
};

export type KitSuspensionRuntime = {
  axles: Map<string, KitAxleRuntime>;
  links: KitLinkRuntime[];
  wheels: KitWheelDef[];
  reset: (chassis: RAPIER.RigidBody) => void;
  syncMeshes: () => void;
};

function add(a: Vec3, b: Vec3): Vec3 {
  return { x: a.x + b.x, y: a.y + b.y, z: a.z + b.z };
}

function sub(a: Vec3, b: Vec3): Vec3 {
  return { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z };
}

function scale(a: Vec3, s: number): Vec3 {
  return { x: a.x * s, y: a.y * s, z: a.z * s };
}

function mid(a: Vec3, b: Vec3): Vec3 {
  return scale(add(a, b), 0.5);
}

function len(a: Vec3): number {
  return Math.hypot(a.x, a.y, a.z);
}

function mountOffset(mounts: MountDef[], id: string, context: string): Vec3 {
  const found = mounts.find((m) => m.id === id);
  if (!found) throw new Error(`${context}: missing mount "${id}"`);
  return found.offset;
}

function resolveLocal(
  ref: MountRef,
  kit: KitDef,
  axles: Map<string, AxleDef>,
  context: string
): { bodyKey: string; local: Vec3 } {
  if (ref.part === "chassis") {
    return { bodyKey: "chassis", local: mountOffset(kit.chassis.mounts, ref.mount, context) };
  }
  const axle = axles.get(ref.part);
  if (!axle) throw new Error(`${context}: unknown part "${ref.part}"`);
  return { bodyKey: axle.id, local: mountOffset(axle.mounts, ref.mount, context) };
}

function worldFromChassis(chassis: RAPIER.RigidBody, local: Vec3): Vec3 {
  const t = chassis.translation();
  const r = chassis.rotation();
  const q = new THREE.Quaternion(r.x, r.y, r.z, r.w);
  const v = new THREE.Vector3(local.x, local.y, local.z).applyQuaternion(q);
  return { x: t.x + v.x, y: t.y + v.y, z: t.z + v.z };
}

function identityQuat(): { x: number; y: number; z: number; w: number } {
  return { x: 0, y: 0, z: 0, w: 1 };
}

function chassisLocalForRef(ref: { bodyKey: string; local: Vec3 }, axles: Map<string, AxleDef>): Vec3 {
  if (ref.bodyKey === "chassis") return ref.local;
  const axle = axles.get(ref.bodyKey);
  if (!axle) throw new Error(`missing axle ${ref.bodyKey}`);
  return add(axle.offset, ref.local);
}

function meshLookRotation(from: Vec3, to: Vec3): THREE.Quaternion {
  const dir = sub(to, from);
  const length = len(dir);
  if (length < 1e-6) return new THREE.Quaternion();
  const up = Math.abs(dir.y / length) > 0.92 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(0, 1, 0);
  const z = new THREE.Vector3(dir.x / length, dir.y / length, dir.z / length);
  const x = new THREE.Vector3().crossVectors(up, z);
  if (x.lengthSq() < 1e-8) x.set(1, 0, 0);
  x.normalize();
  const y = new THREE.Vector3().crossVectors(z, x).normalize();
  return new THREE.Quaternion().setFromRotationMatrix(new THREE.Matrix4().makeBasis(x, y, z));
}

function createKitCollider(world: RAPIER.World, body: RAPIER.RigidBody, desc: RAPIER.ColliderDesc): void {
  world.createCollider(
    desc.setCollisionGroups(KIT_PART_GROUPS).setSolverGroups(KIT_PART_GROUPS).setRestitution(0).setFriction(0.15),
    body
  );
}

/**
 * Build Strategy B kit: axles, spherical links, soft hub plant spheres.
 * Rest poses match BOM mounts so sphericals start at zero error.
 */
export function buildKitSuspension(
  world: RAPIER.World,
  scene: THREE.Scene,
  chassis: RAPIER.RigidBody,
  kit: KitDef
): KitSuspensionRuntime {
  const n = chassis.numColliders();
  for (let i = 0; i < n; i += 1) {
    chassis.collider(i).setCollisionGroups(CHASSIS_GROUPS);
    chassis.collider(i).setSolverGroups(CHASSIS_GROUPS);
  }

  world.integrationParameters.numSolverIterations = Math.max(
    world.integrationParameters.numSolverIterations,
    28
  );
  world.integrationParameters.normalizedAllowedLinearError = Math.min(
    world.integrationParameters.normalizedAllowedLinearError,
    0.0005
  );
  world.integrationParameters.normalizedAllowedLinearError = Math.min(
    world.integrationParameters.normalizedAllowedLinearError,
    0.0005
  );

  const axleDefs = new Map(kit.axles.map((a) => [a.id, a]));
  const axles = new Map<string, KitAxleRuntime>();
  const links: KitLinkRuntime[] = [];
  const bodyByKey = new Map<string, RAPIER.RigidBody>();
  bodyByKey.set("chassis", chassis);

  for (const def of kit.axles) {
    const worldPos = worldFromChassis(chassis, def.offset);
    const body = world.createRigidBody(
      RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(worldPos.x, worldPos.y, worldPos.z)
        .setRotation(identityQuat())
        .setCanSleep(false)
        .setLinearDamping(0.8)
        .setAngularDamping(0.92)
    );
    createKitCollider(
      world,
      body,
      RAPIER.ColliderDesc.cuboid(AXLE_HALF.x, AXLE_HALF.y, AXLE_HALF.z).setMass(AXLE_MASS)
    );
    const shockMounts = def.mounts.filter((m) => m.id.includes("shock")).map((m) => m.offset);
    const mesh = buildScxAxleVisual(AXLE_HALF.x, shockMounts);
    scene.add(mesh);
    axles.set(def.id, { id: def.id, body, mesh, def, restLocal: { ...def.offset } });
    bodyByKey.set(def.id, body);
  }

  // Soft hub plant spheres (ONE vertical plant — not stacked with chassis-ray spring).
  for (const wheel of kit.wheels) {
    const axle = axles.get(wheel.axle);
    if (!axle) throw new Error(`hub sphere: unknown axle ${wheel.axle}`);
    world.createCollider(
      RAPIER.ColliderDesc.ball(wheel.radius)
        .setTranslation(wheel.hubOffset.x, wheel.hubOffset.y, wheel.hubOffset.z)
        .setDensity(0)
        .setFriction(0.05)
        .setRestitution(0)
        .setCollisionGroups(HUB_GROUPS)
        .setSolverGroups(HUB_GROUPS),
      axle.body
    );
  }

  for (const linkDef of kit.links) {
    const from = resolveLocal(linkDef.from, kit, axleDefs, `link ${linkDef.id} from`);
    const to = resolveLocal(linkDef.to, kit, axleDefs, `link ${linkDef.id} to`);
    const fromBody = bodyByKey.get(from.bodyKey);
    const toBody = bodyByKey.get(to.bodyKey);
    if (!fromBody || !toBody) throw new Error(`link ${linkDef.id}: missing body`);

    const fromWorld = worldFromChassis(chassis, chassisLocalForRef(from, axleDefs));
    const toWorld = worldFromChassis(chassis, chassisLocalForRef(to, axleDefs));
    const center = mid(fromWorld, toWorld);
    const length = Math.max(len(sub(toWorld, fromWorld)), 0.05);

    const body = world.createRigidBody(
      RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(center.x, center.y, center.z)
        .setRotation(identityQuat())
        .setCanSleep(false)
        .setLinearDamping(0.3)
        .setAngularDamping(0.6)
    );
    createKitCollider(world, body, RAPIER.ColliderDesc.ball(LINK_RADIUS).setMass(LINK_MASS));

    const anchorLinkFrom = sub(fromWorld, center);
    const anchorLinkTo = sub(toWorld, center);
    world.createImpulseJoint(RAPIER.JointData.spherical(from.local, anchorLinkFrom), fromBody, body, true);
    world.createImpulseJoint(RAPIER.JointData.spherical(anchorLinkTo, to.local), body, toBody, true);

    const mesh = buildScxLinkVisual(length, linkDef.kind === "panhard");
    mesh.position.set(center.x, center.y, center.z);
    mesh.quaternion.copy(meshLookRotation(fromWorld, toWorld));
    scene.add(mesh);
    links.push({ id: linkDef.id, body, mesh, def: linkDef, length });
  }

  const reset = (chassisBody: RAPIER.RigidBody): void => {
    const rot = identityQuat();
    for (const axle of axles.values()) {
      const p = worldFromChassis(chassisBody, axle.restLocal);
      axle.body.setTranslation(p, true);
      axle.body.setRotation(rot, true);
      axle.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
      axle.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
    }
    for (const link of links) {
      const from = resolveLocal(link.def.from, kit, axleDefs, `reset ${link.id}`);
      const to = resolveLocal(link.def.to, kit, axleDefs, `reset ${link.id}`);
      const fromWorld = worldFromChassis(chassisBody, chassisLocalForRef(from, axleDefs));
      const toWorld = worldFromChassis(chassisBody, chassisLocalForRef(to, axleDefs));
      const center = mid(fromWorld, toWorld);
      link.body.setTranslation(center, true);
      link.body.setRotation(rot, true);
      link.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
      link.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
      link.mesh.position.set(center.x, center.y, center.z);
      link.mesh.quaternion.copy(meshLookRotation(fromWorld, toWorld));
    }
  };

  const syncMeshes = (): void => {
    for (const axle of axles.values()) syncRigidBodyToObject(axle.body, axle.mesh);
    for (const link of links) {
      const from = resolveLocal(link.def.from, kit, axleDefs, `sync ${link.id}`);
      const to = resolveLocal(link.def.to, kit, axleDefs, `sync ${link.id}`);
      const fromBody = bodyByKey.get(from.bodyKey)!;
      const toBody = bodyByKey.get(to.bodyKey)!;
      const ft = fromBody.translation();
      const fr = fromBody.rotation();
      const tt = toBody.translation();
      const tr = toBody.rotation();
      const fq = new THREE.Quaternion(fr.x, fr.y, fr.z, fr.w);
      const tq = new THREE.Quaternion(tr.x, tr.y, tr.z, tr.w);
      const fromWorld = new THREE.Vector3(from.local.x, from.local.y, from.local.z)
        .applyQuaternion(fq)
        .add(new THREE.Vector3(ft.x, ft.y, ft.z));
      const toWorld = new THREE.Vector3(to.local.x, to.local.y, to.local.z)
        .applyQuaternion(tq)
        .add(new THREE.Vector3(tt.x, tt.y, tt.z));
      link.mesh.position.copy(fromWorld).add(toWorld).multiplyScalar(0.5);
      link.mesh.quaternion.copy(
        meshLookRotation(
          { x: fromWorld.x, y: fromWorld.y, z: fromWorld.z },
          { x: toWorld.x, y: toWorld.y, z: toWorld.z }
        )
      );
      const dist = fromWorld.distanceTo(toWorld);
      link.mesh.scale.set(1, 1, Math.max(dist, 0.05) / link.length);
    }
  };

  return { axles, links, wheels: kit.wheels, reset, syncMeshes };
}

export function hubWorldPosition(axle: KitAxleRuntime, hubOffset: Vec3): THREE.Vector3 {
  const t = axle.body.translation();
  const r = axle.body.rotation();
  const q = new THREE.Quaternion(r.x, r.y, r.z, r.w);
  const local = new THREE.Vector3(hubOffset.x, hubOffset.y, hubOffset.z).applyQuaternion(q);
  return new THREE.Vector3(t.x + local.x, t.y + local.y, t.z + local.z);
}

export function axleQuaternion(axle: KitAxleRuntime): THREE.Quaternion {
  const r = axle.body.rotation();
  return new THREE.Quaternion(r.x, r.y, r.z, r.w);
}