import RAPIER from "@dimforge/rapier3d-compat";
import * as THREE from "three";
import type { DriveInput } from "../input/Input.ts";
import { CHASSIS_GROUPS } from "../physics/collisionGroups.ts";
import type { PhysicsWorld } from "../physics/PhysicsWorld.ts";
import { syncRigidBodyToObject } from "../physics/sync.ts";
import { applyDrive, createDriveState, createWheelSims, resetDriveState, resetWheelSims, type DriveState, type WheelSim } from "./drive.ts";
import {
  axleQuaternion,
  buildKitSuspension,
  hubWorldPosition,
  type KitSuspensionRuntime,
} from "./kitBodies.ts";
import { applyCoilovers, buildCoilovers, type CoiloverRuntime } from "./kitCoilovers.ts";
import {
  applyHubDrive,
  createHubDriveState,
  resetHubDriveState,
  type HubDriveState,
} from "./kitHubDrive.ts";
import { applyKitLocate, buildKitLocate, type KitLocateRuntime } from "./kitLocate.ts";
import type { KitWheelDef, RigDef } from "./types.ts";
import {
  buildScxChassisVisual,
  buildScxShockVisual,
  syncShockVisual,
  type ShockVisual,
} from "./scxVisuals.ts";

type WheelRuntime = {
  sim: WheelSim;
  mesh: THREE.Mesh;
  kitWheel?: KitWheelDef;
};

const localOffset = new THREE.Vector3();
const suspensionDir = new THREE.Vector3();
const shockFrom = new THREE.Vector3();
const shockTo = new THREE.Vector3();
const shockScratch = new THREE.Vector3();
const shockQ = new THREE.Quaternion();

export class CrawlerVehicle {
  readonly chassisMesh: THREE.Object3D;
  readonly chassisBody: RAPIER.RigidBody;
  private readonly wheels: WheelRuntime[];
  private readonly sims: WheelSim[];
  private readonly driveState: DriveState;
  private readonly hubDriveState: HubDriveState;
  private readonly rig: RigDef;
  private readonly kit: KitSuspensionRuntime | null;
  private readonly locate: KitLocateRuntime | null;
  private readonly coilovers: CoiloverRuntime[];
  private readonly shockVisuals: ShockVisual[];

  constructor(physics: PhysicsWorld, scene: THREE.Scene, rig: RigDef) {
    this.rig = rig;
    const { halfExtents, mass } = rig.chassis;

    const chassisDesc = RAPIER.RigidBodyDesc.dynamic()
      .setTranslation(rig.spawn.x, rig.spawn.y, rig.spawn.z)
      .setCanSleep(false)
      .setCcdEnabled(true)
      .setLinearDamping(0.05)
      .setAngularDamping(0.35);
    this.chassisBody = physics.world.createRigidBody(chassisDesc);
    physics.world.createCollider(
      RAPIER.ColliderDesc.cuboid(halfExtents.x, halfExtents.y, halfExtents.z)
        .setTranslation(0, -0.02, 0)
        .setMass(mass)
        .setFriction(0.02)
        .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
        .setRestitution(0)
        .setCollisionGroups(CHASSIS_GROUPS)
        .setSolverGroups(CHASSIS_GROUPS),
      this.chassisBody
    );

    // Phase 5: multipart SCX10.1 visual — Rapier chassis cuboid unchanged.
    this.chassisMesh = buildScxChassisVisual(halfExtents, rig.kit ?? null);
    scene.add(this.chassisMesh);

    this.kit = rig.kit ? buildKitSuspension(physics.world, scene, this.chassisBody, rig.kit) : null;
    if (this.kit) {
      this.chassisBody.setLinearDamping(0.08);
      this.chassisBody.setAngularDamping(0.45);
    }
    this.locate =
      this.kit && rig.kit
        ? buildKitLocate(scene, this.chassisMesh, this.chassisBody, rig.kit, this.kit.axles)
        : null;
    this.coilovers = this.kit && rig.kit ? buildCoilovers(rig.kit, this.chassisBody, this.kit.axles) : [];
    this.shockVisuals = [];
    for (let i = 0; i < this.coilovers.length; i += 1) {
      const shock = buildScxShockVisual();
      scene.add(shock.root);
      this.shockVisuals.push(shock);
    }

    this.sims = createWheelSims(rig);
    this.driveState = createDriveState();
    this.hubDriveState = createHubDriveState();
    const kitWheels = new Map((rig.kit?.wheels ?? []).map((w) => [w.id, w]));
    this.wheels = this.sims.map((sim, index) => {
      const def = rig.wheels[index];
      if (!def) throw new Error("wheel sim/rig length mismatch");
      const mesh = new THREE.Mesh(
        new THREE.CylinderGeometry(def.radius, def.radius, def.width, 16),
        new THREE.MeshStandardMaterial({ color: 0x222222 })
      );
      scene.add(mesh);
      return { sim, mesh, kitWheel: kitWheels.get(def.id) };
    });
  }

  preStep(world: RAPIER.World, input: DriveInput, dt: number): void {
    if (input.reset) {
      this.reset();
      return;
    }
    if (this.kit) {
      // Phase 4: coilovers + hub Coulomb grip. No chassis-ray spring (would double-plant).
      applyCoilovers(this.chassisBody, this.kit.axles, this.coilovers, dt);
      if (this.locate) {
        applyKitLocate(this.chassisBody, this.kit.axles, this.locate, dt);
      }
      const hubWheels = this.wheels.flatMap((w) => {
        if (!w.kitWheel) return [];
        const axle = this.kit!.axles.get(w.kitWheel.axle);
        if (!axle) return [];
        return [{ def: w.kitWheel, axle }];
      });
      applyHubDrive(world, this.chassisBody, this.kit.axles, hubWheels, this.rig, input, this.hubDriveState, dt);
      for (const wheel of this.wheels) {
        if (!wheel.kitWheel) continue;
        if (wheel.sim.steered) {
          wheel.sim.steer = this.hubDriveState.steer * this.rig.suspension.steerAngle;
        }
        if (wheel.sim.driven && input.throttle !== 0) {
          wheel.sim.spin += (input.throttle * this.rig.maxSpeed * dt) / wheel.sim.radius;
        }
      }
      return;
    }
    applyDrive(world, this.chassisBody, this.sims, this.rig, input, this.driveState, dt);
  }

  reset(): void {
    const { spawn } = this.rig;
    this.chassisBody.setTranslation({ x: spawn.x, y: spawn.y, z: spawn.z }, true);
    this.chassisBody.setRotation({ x: 0, y: 0, z: 0, w: 1 }, true);
    this.chassisBody.setLinvel({ x: 0, y: 0, z: 0 }, true);
    this.chassisBody.setAngvel({ x: 0, y: 0, z: 0 }, true);
    this.kit?.reset(this.chassisBody);
    resetWheelSims(this.sims);
    resetDriveState(this.driveState);
    resetHubDriveState(this.hubDriveState);
  }

  syncMeshes(): void {
    syncRigidBodyToObject(this.chassisBody, this.chassisMesh);
    this.kit?.syncMeshes();
    if (this.locate && this.kit) {
      this.locate.syncMeshes(this.chassisBody, this.kit.axles);
    }
    this.syncShockMeshes();
    for (const wheel of this.wheels) {
      const { sim, mesh, kitWheel } = wheel;
      if (this.kit && kitWheel) {
        const axle = this.kit.axles.get(kitWheel.axle);
        if (!axle) throw new Error(`missing axle "${kitWheel.axle}" for wheel ${kitWheel.id}`);
        mesh.position.copy(hubWorldPosition(axle, kitWheel.hubOffset));
        mesh.quaternion.copy(axleQuaternion(axle));
        mesh.rotateY(sim.steer);
        mesh.rotateZ(Math.PI / 2);
        mesh.rotateY(sim.spin);
        continue;
      }
      localOffset.set(sim.offset.x, sim.offset.y, sim.offset.z).applyQuaternion(this.chassisMesh.quaternion);
      suspensionDir.set(sim.castX, sim.castY, sim.castZ);
      if (suspensionDir.lengthSq() < 0.5) suspensionDir.set(0, -1, 0).applyQuaternion(this.chassisMesh.quaternion);
      const drop = Math.max(0, sim.suspensionLength - sim.radius);
      mesh.position.copy(this.chassisMesh.position).add(localOffset).addScaledVector(suspensionDir, drop);
      mesh.quaternion.copy(this.chassisMesh.quaternion);
      mesh.rotateY(sim.steer);
      mesh.rotateZ(Math.PI / 2);
      mesh.rotateY(sim.spin);
    }
  }

  private syncShockMeshes(): void {
    if (!this.kit || this.shockVisuals.length === 0) return;
    for (let i = 0; i < this.coilovers.length; i += 1) {
      const coil = this.coilovers[i]!;
      const visual = this.shockVisuals[i];
      if (!visual) continue;
      const axle = this.kit.axles.get(coil.axleId);
      if (!axle) continue;
      const ct = this.chassisBody.translation();
      const cr = this.chassisBody.rotation();
      shockQ.set(cr.x, cr.y, cr.z, cr.w);
      shockScratch.set(coil.chassisMount.x, coil.chassisMount.y, coil.chassisMount.z).applyQuaternion(shockQ);
      shockFrom.set(ct.x + shockScratch.x, ct.y + shockScratch.y, ct.z + shockScratch.z);
      const at = axle.body.translation();
      const ar = axle.body.rotation();
      shockQ.set(ar.x, ar.y, ar.z, ar.w);
      shockScratch.set(coil.axleMount.x, coil.axleMount.y, coil.axleMount.z).applyQuaternion(shockQ);
      shockTo.set(at.x + shockScratch.x, at.y + shockScratch.y, at.z + shockScratch.z);
      syncShockVisual(visual, shockFrom, shockTo);
    }
  }

  chassisPosition(): THREE.Vector3 {
    return this.chassisMesh.position;
  }

  kitSuspension(): KitSuspensionRuntime | null {
    return this.kit;
  }

  /** Lowest hub underside Y — settle gap metric. */
  minHubClearance(): number {
    if (!this.kit) return Number.NaN;
    let min = Infinity;
    for (const wheel of this.wheels) {
      if (!wheel.kitWheel) continue;
      const axle = this.kit.axles.get(wheel.kitWheel.axle);
      if (!axle) continue;
      const hub = hubWorldPosition(axle, wheel.kitWheel.hubOffset);
      min = Math.min(min, hub.y - wheel.kitWheel.radius);
    }
    return min;
  }
}
