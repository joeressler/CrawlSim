import RAPIER from "@dimforge/rapier3d-compat";
import * as THREE from "three";
import { WORLD_GROUPS } from "../physics/collisionGroups.ts";
import { addTrailLights } from "../render/lights.ts";
import type { PhysicsWorld } from "../physics/PhysicsWorld.ts";
import type { Vec3 } from "../vehicles/types.ts";

function worldCollider(desc: RAPIER.ColliderDesc): RAPIER.ColliderDesc {
  return desc.setCollisionGroups(WORLD_GROUPS).setSolverGroups(WORLD_GROUPS);
}

type CuboidHalf = { x: number; y: number; z: number };
type CuboidPos = { x: number; y: number; z: number };

type FixedCuboidOpts = {
  half: CuboidHalf;
  pos: CuboidPos;
  rot?: THREE.Quaternion;
  friction: number;
  material: THREE.Material;
};

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
    this.createRockCourse(physics);
  }

  /** Fixed world cuboid: collider + matching mesh. */
  private addFixedCuboid(physics: PhysicsWorld, opts: FixedCuboidOpts): void {
    const { half, pos, rot, friction, material } = opts;
    const bodyDesc = RAPIER.RigidBodyDesc.fixed().setTranslation(pos.x, pos.y, pos.z);
    if (rot) {
      bodyDesc.setRotation({ x: rot.x, y: rot.y, z: rot.z, w: rot.w });
    }
    const body = physics.world.createRigidBody(bodyDesc);
    physics.world.createCollider(
      worldCollider(
        RAPIER.ColliderDesc.cuboid(half.x, half.y, half.z)
          .setFriction(friction)
          .setRestitution(0)
      ),
      body
    );
    const mesh = new THREE.Mesh(
      new THREE.BoxGeometry(half.x * 2, half.y * 2, half.z * 2),
      material
    );
    mesh.position.set(pos.x, pos.y, pos.z);
    if (rot) mesh.quaternion.copy(rot);
    this.scene.add(mesh);
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
    // ~1.3R / ~2.6R / ~4R (r=0.06). Sharp cube lips pin hub spheres on the
    // top edge; each riser gets a 45° nose so tires can walk the face.
    const heights = [0.08, 0.16, 0.26];
    const laneX = -6.5;
    const startZ = 1.4;
    const stepPitch = 1.4;
    const noseQuat = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 4);
    const addStep = (h: number, z: number, prevH: number): void => {
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

      const rise = h - prevH;
      if (rise > 0.01) {
        // Thin 45° slab on the +Z approach face (drive is toward −Z).
        const halfThick = 0.02;
        const halfSlope = (rise * Math.SQRT2) / 2;
        const faceZ = z + halfZ;
        const noseY = prevH + rise / 2;
        const noseZ = faceZ + rise / 2;
        const noseBody = physics.world.createRigidBody(
          RAPIER.RigidBodyDesc.fixed()
            .setTranslation(laneX, noseY, noseZ)
            .setRotation({ x: noseQuat.x, y: noseQuat.y, z: noseQuat.z, w: noseQuat.w })
        );
        physics.world.createCollider(
          worldCollider(
            RAPIER.ColliderDesc.cuboid(halfX, halfSlope, halfThick)
              .setFriction(0.95)
              .setRestitution(0)
          ),
          noseBody
        );
        const noseMesh = new THREE.Mesh(
          new THREE.BoxGeometry(halfX * 2, halfSlope * 2, halfThick * 2),
          mat
        );
        noseMesh.position.set(laneX, noseY, noseZ);
        noseMesh.quaternion.copy(noseQuat);
        this.scene.add(noseMesh);
      }
    };
    heights.forEach((h, i) => addStep(h, startZ - i * stepPitch, i === 0 ? 0 : heights[i - 1]!));
    const topZ = startZ - heights.length * stepPitch;
    addStep(heights[2]!, topZ, heights[2]!);
    [...heights].reverse().forEach((h, i, arr) => {
      const prev = i === 0 ? heights[2]! : arr[i - 1]!;
      addStep(h, topZ - (i + 1) * stepPitch, prev);
    });
  }

  /**
   * Short rock lane on +X (mirrors stairs). Heights in tire radii (R=0.06):
   * entry ~0.5–1R, articulation ~1.5–2R, mid shelf ~2.5R with 45° nose.
   */
  private createRockCourse(physics: PhysicsWorld): void {
    const mat = new THREE.MeshStandardMaterial({ color: 0x6e6558 });
    const laneX = 6.5;
    const rock = (
      half: CuboidHalf,
      pos: CuboidPos,
      euler?: { x: number; y: number; z: number },
      friction = 0.9
    ): void => {
      const rot = euler
        ? new THREE.Quaternion().setFromEuler(new THREE.Euler(euler.x, euler.y, euler.z))
        : undefined;
      this.addFixedCuboid(physics, { half, pos, rot, friction, material: mat });
    };

    // Entry scatter (~0.5–1R) — staggered left/right toward −Z.
    rock({ x: 0.18, y: 0.02, z: 0.16 }, { x: laneX - 0.35, y: 0.02, z: 2.2 }, { x: 0.12, y: 0.2, z: 0.08 });
    rock({ x: 0.14, y: 0.025, z: 0.12 }, { x: laneX + 0.4, y: 0.025, z: 1.85 }, { x: -0.1, y: -0.25, z: 0.05 });
    rock({ x: 0.2, y: 0.03, z: 0.14 }, { x: laneX - 0.15, y: 0.03, z: 1.45 }, { x: 0.08, y: 0.15, z: -0.1 });
    rock({ x: 0.16, y: 0.028, z: 0.18 }, { x: laneX + 0.35, y: 0.028, z: 1.1 }, { x: -0.15, y: 0.3, z: 0.12 });

    // Articulation gate: one-wheel-high vs low (peak ~1.5–2R), gap ≤ track.
    rock({ x: 0.22, y: 0.055, z: 0.2 }, { x: laneX - 0.12, y: 0.055, z: 0.55 }, { x: 0.18, y: -0.2, z: 0.1 });
    rock({ x: 0.2, y: 0.03, z: 0.18 }, { x: laneX + 0.14, y: 0.03, z: 0.35 }, { x: -0.12, y: 0.25, z: -0.08 });
    rock({ x: 0.18, y: 0.05, z: 0.16 }, { x: laneX + 0.08, y: 0.05, z: -0.05 }, { x: 0.2, y: 0.1, z: 0.15 });
    rock({ x: 0.24, y: 0.035, z: 0.14 }, { x: laneX - 0.2, y: 0.035, z: -0.25 }, { x: -0.08, y: -0.3, z: 0.05 });

    // Mid shelf (~2.5R = 0.15) with 45° approach nose.
    const shelfH = 0.15;
    const shelfHalf = { x: 0.55, y: shelfH / 2, z: 0.4 };
    const shelfZ = -0.95;
    rock(shelfHalf, { x: laneX, y: shelfHalf.y, z: shelfZ });
    {
      const rise = shelfH;
      const halfThick = 0.02;
      const halfSlope = (rise * Math.SQRT2) / 2;
      const noseQuat = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 4);
      const faceZ = shelfZ + shelfHalf.z;
      this.addFixedCuboid(physics, {
        half: { x: shelfHalf.x, y: halfSlope, z: halfThick },
        pos: { x: laneX, y: rise / 2, z: faceZ + rise / 2 },
        rot: noseQuat,
        friction: 0.95,
        material: mat,
      });
    }
    // Off-camber slot rock beside shelf (intentional articulation).
    rock({ x: 0.16, y: 0.045, z: 0.2 }, { x: laneX + 0.72, y: 0.045, z: -0.85 }, { x: 0.25, y: 0.35, z: 0.1 });

    // Exit scatter — descend toward flat.
    rock({ x: 0.2, y: 0.04, z: 0.16 }, { x: laneX - 0.25, y: 0.04, z: -1.55 }, { x: -0.18, y: 0.15, z: 0.08 });
    rock({ x: 0.18, y: 0.03, z: 0.2 }, { x: laneX + 0.3, y: 0.03, z: -1.9 }, { x: 0.1, y: -0.22, z: -0.12 });
    rock({ x: 0.14, y: 0.022, z: 0.14 }, { x: laneX - 0.1, y: 0.022, z: -2.3 }, { x: 0.15, y: 0.28, z: 0.05 });
    rock({ x: 0.16, y: 0.018, z: 0.12 }, { x: laneX + 0.25, y: 0.018, z: -2.65 }, { x: -0.1, y: -0.18, z: 0.1 });
  }
}
