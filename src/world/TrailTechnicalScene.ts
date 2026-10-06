import RAPIER from "@dimforge/rapier3d-compat";
import * as THREE from "three";
import { WORLD_GROUPS } from "../physics/collisionGroups.ts";
import { addTrailLights } from "../render/lights.ts";
import type { Vec3 } from "../vehicles/types.ts";
import type { SceneContext, WorldScene } from "./WorldScene.ts";

function worldCollider(desc: RAPIER.ColliderDesc): RAPIER.ColliderDesc {
  return desc.setCollisionGroups(WORLD_GROUPS).setSolverGroups(WORLD_GROUPS);
}

/**
 * Technical trail: one forward lane (drive toward -Z) with obstacles in
 * sequence along the path — not side-by-side playground pads.
 */
export class TrailTechnicalScene implements WorldScene {
  readonly id = "trail-technical" as const;
  readonly scene: THREE.Scene;
  readonly spawn: Vec3;
  private built = false;
  private readonly worldBodies: RAPIER.RigidBody[] = [];

  constructor(defaultSpawn: Vec3) {
    // Start at the +Z end of the course so the truck drives forward through stations.
    this.spawn = { x: 0, y: defaultSpawn.y, z: 6.2 };
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x9ac7ff);
    addTrailLights(this.scene);
  }

  activate(ctx: SceneContext): void {
    if (this.built) return;
    this.createGround(ctx.physics.world);
    this.createSequentialCourse(ctx.physics.world);
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
    // Long corridor along Z so the sequential course fits.
    const body = this.trackBody(world.createRigidBody(RAPIER.RigidBodyDesc.fixed()));
    world.createCollider(
      worldCollider(
        RAPIER.ColliderDesc.cuboid(6, 0.25, 14)
          .setTranslation(0, -0.25, -2)
          .setFriction(0.6)
          .setRestitution(0)
      ),
      body
    );
    const mesh = new THREE.Mesh(
      new THREE.BoxGeometry(12, 0.5, 28),
      new THREE.MeshStandardMaterial({ color: 0x557a4f })
    );
    mesh.position.set(0, -0.25, -2);
    this.scene.add(mesh);
  }

  /**
   * Stations spaced along -Z. Path centerline stays near x=0; lateral
   * offsets are only for articulation within a station, not alternate lanes.
   */
  private createSequentialCourse(world: RAPIER.World): void {
    // 1) Entry washboard — thin ridges the truck crosses one after another.
    const ridgeZs = [4.6, 4.15, 3.7];
    ridgeZs.forEach((z, i) => {
      const h = 0.035 + i * 0.012;
      this.addBlock(world, { x: 0.55, y: h / 2, z: 0.12 }, { x: 0, y: h / 2, z }, 0x7e6b55, 0.9);
    });

    // 2) Low step-up ledge.
    this.addBlock(world, { x: 0.7, y: 0.05, z: 0.55 }, { x: 0, y: 0.05, z: 2.7 }, 0x876f58, 0.9);

    // 3) Articulation gate — left high / right low, still on the same lane.
    this.addBlock(world, { x: 0.28, y: 0.07, z: 0.28 }, { x: -0.22, y: 0.07, z: 1.35 }, 0x6e6558, 0.92);
    this.addBlock(world, { x: 0.26, y: 0.03, z: 0.26 }, { x: 0.24, y: 0.03, z: 1.05 }, 0x6e6558, 0.92);
    this.addBlock(world, { x: 0.24, y: 0.055, z: 0.22 }, { x: 0.18, y: 0.055, z: 0.65 }, 0x74685a, 0.92);
    this.addBlock(world, { x: 0.26, y: 0.028, z: 0.24 }, { x: -0.2, y: 0.028, z: 0.35 }, 0x74685a, 0.92);

    // 4) Climbing ramp (pitch about X; approach from +Z).
    {
      const pitch = 0.2;
      const half = { x: 0.85, y: 0.1, z: 1.1 };
      const rot = new THREE.Quaternion().setFromEuler(new THREE.Euler(pitch, 0, 0));
      const nearTopY = half.y * Math.cos(pitch) - half.z * Math.sin(pitch);
      const rampY = 0.06 - nearTopY;
      this.addBlock(world, half, { x: 0, y: rampY, z: -0.85 }, 0x8f7559, 0.9, rot);
    }

    // 5) Three descending steps after the ramp crest.
    const stepHeights = [0.18, 0.12, 0.07];
    const stepStartZ = -2.35;
    stepHeights.forEach((h, i) => {
      const halfY = h / 2;
      this.addBlock(
        world,
        { x: 0.75, y: halfY, z: 0.4 },
        { x: 0, y: halfY, z: stepStartZ - i * 0.95 },
        0x94795f,
        0.9
      );
    });

    // 6) Off-camber slab — single obstacle further down the path.
    {
      const roll = 0.28;
      const rot = new THREE.Quaternion().setFromEuler(new THREE.Euler(0.06, 0, roll));
      this.addBlock(world, { x: 0.95, y: 0.08, z: 0.9 }, { x: 0, y: 0.14, z: -5.4 }, 0x8a7054, 0.9, rot);
    }

    // 7) Finish shelf with a short approach nose.
    {
      const shelfH = 0.14;
      const half = { x: 0.7, y: shelfH / 2, z: 0.55 };
      const shelfZ = -7.2;
      this.addBlock(world, half, { x: 0, y: half.y, z: shelfZ }, 0x9a6b3f, 0.95);
      const rise = shelfH;
      const halfThick = 0.02;
      const halfSlope = (rise * Math.SQRT2) / 2;
      const noseQuat = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 4);
      const faceZ = shelfZ + half.z;
      this.addBlock(
        world,
        { x: half.x, y: halfSlope, z: halfThick },
        { x: 0, y: rise / 2, z: faceZ + rise / 2 },
        0x9a6b3f,
        0.95,
        noseQuat
      );
    }

    // Soft exit pad past the shelf so the lane reads as a finished course.
    this.addBlock(world, { x: 0.5, y: 0.02, z: 0.35 }, { x: 0, y: 0.02, z: -8.4 }, 0x7e6b55, 0.85);
  }
}