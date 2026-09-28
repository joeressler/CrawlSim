import RAPIER from "@dimforge/rapier3d-compat";
import * as THREE from "three";
import { WORLD_GROUPS } from "../physics/collisionGroups.ts";
import { addTrailLights } from "../render/lights.ts";
import type { PhysicsWorld } from "../physics/PhysicsWorld.ts";
import type { Vec3 } from "../vehicles/types.ts";

function worldCollider(desc: RAPIER.ColliderDesc): RAPIER.ColliderDesc {
  return desc.setCollisionGroups(WORLD_GROUPS).setSolverGroups(WORLD_GROUPS);
}

export class TrailScene {
  readonly scene: THREE.Scene;
  readonly spawn: Vec3;

  constructor(physics: PhysicsWorld, spawn: Vec3) {
    this.spawn = spawn;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x87b5ff);
    addTrailLights(this.scene);
    this.createGround(physics);
    this.createRamp(physics);
    this.createLedge(physics);
    this.createObstacleCourse(physics);
  }

  private createGround(physics: PhysicsWorld): void {
    const groundBody = physics.world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    physics.world.createCollider(
      worldCollider(
        RAPIER.ColliderDesc.cuboid(20, 0.25, 20)
          .setTranslation(0, -0.25, 0)
          .setFriction(0.55)
          .setRestitution(0)
      ),
      groundBody
    );
    const groundMesh = new THREE.Mesh(
      new THREE.BoxGeometry(40, 0.5, 40),
      new THREE.MeshStandardMaterial({ color: 0x4a7c59 })
    );
    groundMesh.position.y = -0.25;
    this.scene.add(groundMesh);
  }

  private createRamp(physics: PhysicsWorld): void {
    const pitch = 0.28;
    const half = { x: 3, y: 0.15, z: 4 };
    const rotation = new THREE.Quaternion().setFromEuler(new THREE.Euler(pitch, 0, 0));
    const nearTopY = half.y * Math.cos(pitch) - half.z * Math.sin(pitch);
    const rampY = 0.08 - nearTopY;
    const rampZ = -1;
    const ramp = physics.world.createRigidBody(
      RAPIER.RigidBodyDesc.fixed().setTranslation(0, rampY, rampZ).setRotation(rotation)
    );
    physics.world.createCollider(
      worldCollider(RAPIER.ColliderDesc.cuboid(half.x, half.y, half.z).setFriction(0.7).setRestitution(0)),
      ramp
    );
    const rampMesh = new THREE.Mesh(
      new THREE.BoxGeometry(half.x * 2, half.y * 2, half.z * 2),
      new THREE.MeshStandardMaterial({ color: 0x8b7355 })
    );
    rampMesh.position.set(0, rampY, rampZ);
    rampMesh.quaternion.copy(rotation);
    this.scene.add(rampMesh);
  }

  /** Box about one tire radius tall, off the ramp line. Grip has to crest it. */
  private createLedge(physics: PhysicsWorld): void {
    const height = 0.22;
    const half = { x: 1.25, y: height / 2, z: 1.25 };
    const position = { x: 4.2, y: half.y, z: 2 };
    const body = physics.world.createRigidBody(
      RAPIER.RigidBodyDesc.fixed().setTranslation(position.x, position.y, position.z)
    );
    physics.world.createCollider(
      worldCollider(RAPIER.ColliderDesc.cuboid(half.x, half.y, half.z).setFriction(0.9).setRestitution(0)),
      body
    );
    const mesh = new THREE.Mesh(
      new THREE.BoxGeometry(half.x * 2, half.y * 2, half.z * 2),
      new THREE.MeshStandardMaterial({ color: 0x9a6b3f })
    );
    mesh.position.set(position.x, position.y, position.z);
    this.scene.add(mesh);
  }

  /** Three-tier stairs up, then down, on −X. */
  private createObstacleCourse(physics: PhysicsWorld): void {
    const mat = new THREE.MeshStandardMaterial({ color: 0x7a5a3a });
    const halfX = 1.2;
    const halfZ = 0.65;
    const heights = [0.18, 0.3, 0.42];
    const laneX = -6.5;
    const startZ = 1.4;
    const stepPitch = 1.4;
    const addStep = (h: number, z: number): void => {
      const halfY = h / 2;
      const body = physics.world.createRigidBody(
        RAPIER.RigidBodyDesc.fixed().setTranslation(laneX, halfY, z)
      );
      physics.world.createCollider(
        worldCollider(
          RAPIER.ColliderDesc.cuboid(halfX, halfY, halfZ)
            .setFriction(0.9)
            .setRestitution(0)
        ),
        body
      );
      const mesh = new THREE.Mesh(new THREE.BoxGeometry(halfX * 2, h, halfZ * 2), mat);
      mesh.position.set(laneX, halfY, z);
      this.scene.add(mesh);
    };
    heights.forEach((h, i) => addStep(h, startZ - i * stepPitch));
    const topZ = startZ - heights.length * stepPitch;
    addStep(heights[2]!, topZ);
    [...heights].reverse().forEach((h, i) => addStep(h, topZ - (i + 1) * stepPitch));
  }
}