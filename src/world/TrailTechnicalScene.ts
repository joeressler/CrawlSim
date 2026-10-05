import RAPIER from "@dimforge/rapier3d-compat";
import * as THREE from "three";
import { WORLD_GROUPS } from "../physics/collisionGroups.ts";
import { addTrailLights } from "../render/lights.ts";
import type { Vec3 } from "../vehicles/types.ts";
import type { SceneContext, WorldScene } from "./WorldScene.ts";

function worldCollider(desc: RAPIER.ColliderDesc): RAPIER.ColliderDesc {
  return desc.setCollisionGroups(WORLD_GROUPS).setSolverGroups(WORLD_GROUPS);
}

export class TrailTechnicalScene implements WorldScene {
  readonly id = "trail-technical" as const;
  readonly scene: THREE.Scene;
  readonly spawn: Vec3;
  private built = false;
  private readonly worldBodies: RAPIER.RigidBody[] = [];

  constructor(defaultSpawn: Vec3) {
    this.spawn = { x: defaultSpawn.x, y: defaultSpawn.y, z: defaultSpawn.z + 2.5 };
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x9ac7ff);
    addTrailLights(this.scene);
  }

  activate(ctx: SceneContext): void {
    if (this.built) return;
    this.createGround(ctx.physics.world);
    this.createRidgeLine(ctx.physics.world);
    this.createOffsetRamps(ctx.physics.world);
    this.built = true;
  }

  deactivate(_ctx: SceneContext): void {
    // No scene subscriptions to pause.
  }

  dispose(ctx: SceneContext): void {
    for (const body of this.worldBodies) {
      if (!body.isValid()) continue;
      ctx.physics.world.removeRigidBody(body);
    }
    this.worldBodies.length = 0;
    this.scene.clear();
    addTrailLights(this.scene);
    this.built = false;
  }

  private trackBody(body: RAPIER.RigidBody): RAPIER.RigidBody {
    this.worldBodies.push(body);
    return body;
  }

  private addBlock(
    world: RAPIER.World,
    half: { x: number; y: number; z: number },
    pos: { x: number; y: number; z: number },
    color: number,
    friction = 0.85,
    rotation?: THREE.Quaternion
  ): void {
    const bodyDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(pos.x, pos.y, pos.z);
    if (rotation) {
      bodyDesc.setRotation({ x: rotation.x, y: rotation.y, z: rotation.z, w: rotation.w });
    }
    const body = this.trackBody(world.createRigidBody(bodyDesc));
    world.createCollider(
      worldCollider(RAPIER.ColliderDesc.cuboid(half.x, half.y, half.z).setFriction(friction).setRestitution(0)),
      body
    );
    const mesh = new THREE.Mesh(
      new THREE.BoxGeometry(half.x * 2, half.y * 2, half.z * 2),
      new THREE.MeshStandardMaterial({ color })
    );
    mesh.position.set(pos.x, pos.y, pos.z);
    if (rotation) mesh.quaternion.copy(rotation);
    this.scene.add(mesh);
  }

  private createGround(world: RAPIER.World): void {
    const body = this.trackBody(world.createRigidBody(RAPIER.RigidBodyDesc.fixed()));
    world.createCollider(
      worldCollider(
        RAPIER.ColliderDesc.cuboid(20, 0.25, 20)
          .setTranslation(0, -0.25, 0)
          .setFriction(0.6)
          .setRestitution(0)
      ),
      body
    );
    const mesh = new THREE.Mesh(
      new THREE.BoxGeometry(40, 0.5, 40),
      new THREE.MeshStandardMaterial({ color: 0x557a4f })
    );
    mesh.position.set(0, -0.25, 0);
    this.scene.add(mesh);
  }

  private createRidgeLine(world: RAPIER.World): void {
    this.addBlock(world, { x: 0.8, y: 0.06, z: 1.3 }, { x: -1.4, y: 0.06, z: -0.2 }, 0x7e6b55);
    this.addBlock(world, { x: 0.8, y: 0.1, z: 1.3 }, { x: 0, y: 0.1, z: -0.7 }, 0x876f58);
    this.addBlock(world, { x: 0.8, y: 0.14, z: 1.3 }, { x: 1.4, y: 0.14, z: -1.25 }, 0x94795f);
  }

  private createOffsetRamps(world: RAPIER.World): void {
    const leftRot = new THREE.Quaternion().setFromEuler(new THREE.Euler(0.22, 0.18, 0));
    this.addBlock(
      world,
      { x: 1.6, y: 0.12, z: 1.6 },
      { x: -4.4, y: 0.2, z: 1.8 },
      0x8f7559,
      0.9,
      leftRot
    );

    const rightRot = new THREE.Quaternion().setFromEuler(new THREE.Euler(0.2, -0.22, 0));
    this.addBlock(
      world,
      { x: 1.6, y: 0.12, z: 1.6 },
      { x: 4.2, y: 0.18, z: 1.2 },
      0x8a7054,
      0.9,
      rightRot
    );
  }
}
