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
import type { KitWheelDef, RigDef } from "./types.ts";

type WheelRuntime = {
  sim: WheelSim;
  mesh: THREE.Mesh;
  /** Strategy B: hub on a kit axle. */
  kitWheel?: KitWheelDef;
};

const localOffset = new THREE.Vector3();
const suspensionDir = new THREE.Vector3();

export class CrawlerVehicle {
  readonly chassisMesh: THREE.Mesh;
  readonly chassisBody: RAPIER.RigidBody;
  private readonly wheels: WheelRuntime[];
  private readonly sims: WheelSim[];
  private readonly driveState: DriveState;
  private readonly rig: RigDef;
  private readonly kit: KitSuspensionRuntime | null;

  constructor(physics: PhysicsWorld, scene: THREE.Scene, rig: RigDef) {
    this.rig = rig;
    const { halfExtents, mass } = rig.chassis;

    const chassisDesc = RAPIER.RigidBodyDesc.dynamic()
      .setTranslation(rig.spawn.x, rig.spawn.y, rig.spawn.z)
      .setCanSleep(false)
      .setLinearDamping(0.05)
      .setAngularDamping(0.35);
    this.chassisBody = physics.world.createRigidBody(chassisDesc);
    physics.world.createCollider(
      RAPIER.ColliderDesc.cuboid(halfExtents.x, halfExtents.y, halfExtents.z)
        .setMass(mass)
        .setFriction(0.02)
        .setFrictionCombineRule(RAPIER.CoefficientCombineRule.Min)
        .setRestitution(0)
        .setCollisionGroups(CHASSIS_GROUPS)
        .setSolverGroups(CHASSIS_GROUPS),
      this.chassisBody
    );

    this.chassisMesh = new THREE.Mesh(
      new THREE.BoxGeometry(halfExtents.x * 2, halfExtents.y * 2, halfExtents.z * 2),
      new THREE.MeshStandardMaterial({ color: 0x1f4e79 })
    );
    scene.add(this.chassisMesh);

    this.kit = rig.kit ? buildKitSuspension(physics.world, scene, this.chassisBody, rig.kit) : null;

    this.sims = createWheelSims(rig);
    this.driveState = createDriveState();
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
    // Phase 2a: kit sphericals + temporary g0 hold. Chassis-ray drive fights the
    // articulation (may return in Phase 3 with coilovers). Throttle is a no-op here.
    if (!this.kit) {
      applyDrive(world, this.chassisBody, this.sims, this.rig, input, this.driveState, dt);
    } else if (input.throttle !== 0 || input.steer !== 0) {
      // Keep steer/spin visuals alive lightly without ray forces.
      for (const wheel of this.wheels) {
        if (wheel.sim.steered) wheel.sim.steer = input.steer * this.rig.suspension.steerAngle;
        if (wheel.sim.driven && input.throttle !== 0) {
          wheel.sim.spin += (input.throttle * this.rig.maxSpeed * dt) / wheel.sim.radius;
        }
      }
    }
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
  }

  syncMeshes(): void {
    syncRigidBodyToObject(this.chassisBody, this.chassisMesh);
    this.kit?.syncMeshes();
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

  chassisPosition(): THREE.Vector3 {
    return this.chassisMesh.position;
  }

  /** Expose kit runtime for settle metrics / later phases. */
  kitSuspension(): KitSuspensionRuntime | null {
    return this.kit;
  }
}