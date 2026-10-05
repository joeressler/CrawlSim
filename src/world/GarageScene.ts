import RAPIER from "@dimforge/rapier3d-compat";
import * as THREE from "three";
import { WORLD_GROUPS } from "../physics/collisionGroups.ts";
import type { Vec3 } from "../vehicles/types.ts";
import type { SceneContext, WorldScene } from "./WorldScene.ts";

function worldCollider(desc: RAPIER.ColliderDesc): RAPIER.ColliderDesc {
  return desc.setCollisionGroups(WORLD_GROUPS).setSolverGroups(WORLD_GROUPS);
}

export class GarageScene implements WorldScene {
  readonly id = "garage" as const;
  readonly scene: THREE.Scene;
  readonly spawn: Vec3;
  private built = false;
  private readonly worldBodies: RAPIER.RigidBody[] = [];
  private readonly rcFocus = new THREE.Vector3(0, 0.22, -2);

  constructor(defaultSpawn: Vec3) {
    this.spawn = { x: 0, y: defaultSpawn.y, z: -2 };
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0xcfd6df);
  }

  activate(ctx: SceneContext): void {
    if (this.built) return;
    this.addGarageLights();
    const floor = this.trackBody(ctx.physics.world.createRigidBody(RAPIER.RigidBodyDesc.fixed()));
    ctx.physics.world.createCollider(
      worldCollider(
        RAPIER.ColliderDesc.cuboid(10, 0.2, 10)
          .setTranslation(0, -0.2, 0)
          .setFriction(0.7)
          .setRestitution(0)
      ),
      floor
    );

    const floorMesh = new THREE.Mesh(
      new THREE.BoxGeometry(20, 0.4, 20),
      new THREE.MeshStandardMaterial({ color: 0x7b828c, roughness: 0.92, metalness: 0.04 })
    );
    floorMesh.position.set(0, -0.2, 0);
    this.scene.add(floorMesh);

    const pad = this.trackBody(
      ctx.physics.world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(0, 0.06, -2))
    );
    ctx.physics.world.createCollider(
      worldCollider(RAPIER.ColliderDesc.cuboid(1.8, 0.06, 1.8).setFriction(0.8).setRestitution(0)),
      pad
    );
    const padMesh = new THREE.Mesh(
      new THREE.BoxGeometry(3.6, 0.12, 3.6),
      new THREE.MeshStandardMaterial({ color: 0x3f4752, roughness: 0.75, metalness: 0.2 })
    );
    padMesh.position.set(0, 0.06, -2);
    this.scene.add(padMesh);

    this.addBayStriping();
    this.addCatalogBackdrop();
    this.addGarageBackWall();
    this.addWorkbench();
    this.addShelfWall();
    this.addToolChest();
    this.addTireStack();
    this.addStorageCrates();

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
    this.built = false;
  }

  private trackBody(body: RAPIER.RigidBody): RAPIER.RigidBody {
    this.worldBodies.push(body);
    return body;
  }

  private addGarageLights(): void {
    const hemi = new THREE.HemisphereLight(0xeef4ff, 0x545860, 0.38);
    this.scene.add(hemi);

    const ambient = new THREE.AmbientLight(0xffffff, 0.12);
    this.scene.add(ambient);

    const key = new THREE.SpotLight(0xffffff, 3.15, 24, 0.44, 0.4, 1.15);
    key.position.set(2.1, 3.2, 0.2);
    key.target.position.copy(this.rcFocus);
    this.scene.add(key);
    this.scene.add(key.target);

    const fill = new THREE.SpotLight(0xfff3dc, 0.5, 18, 0.6, 0.55, 1.35);
    fill.position.set(-2.2, 1.8, -3.6);
    fill.target.position.set(this.rcFocus.x - 0.25, this.rcFocus.y + 0.08, this.rcFocus.z - 0.35);
    this.scene.add(fill);
    this.scene.add(fill.target);

    const rim = new THREE.SpotLight(0xcfe1ff, 1.15, 14, 0.38, 0.35, 1.2);
    rim.position.set(-2.7, 2.1, -0.85);
    rim.target.position.set(this.rcFocus.x + 0.2, this.rcFocus.y + 0.12, this.rcFocus.z + 0.05);
    this.scene.add(rim);
    this.scene.add(rim.target);
  }

  private addCatalogBackdrop(): void {
    const sweepMat = new THREE.MeshStandardMaterial({ color: 0xe6ebf3, roughness: 0.92, metalness: 0.02 });
    const sweep = new THREE.Mesh(new THREE.BoxGeometry(4.8, 2.7, 0.08), sweepMat);
    sweep.position.set(0, 1.24, -3.75);
    this.scene.add(sweep);

    const sideWingMat = new THREE.MeshStandardMaterial({ color: 0xdce3ec, roughness: 0.9, metalness: 0.03 });
    const wingLeft = new THREE.Mesh(new THREE.BoxGeometry(0.08, 2.2, 2.15), sideWingMat);
    wingLeft.position.set(-2.36, 1.0, -2.72);
    this.scene.add(wingLeft);

    const wingRight = new THREE.Mesh(new THREE.BoxGeometry(0.08, 2.2, 2.15), sideWingMat);
    wingRight.position.set(2.36, 1.0, -2.72);
    this.scene.add(wingRight);
  }

  private addBayStriping(): void {
    const stripeMat = new THREE.MeshStandardMaterial({ color: 0xffcc4d, roughness: 0.72, metalness: 0.06 });
    const stripeA = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.003, 4.2), stripeMat);
    stripeA.position.set(-2.0, 0.001, -2);
    this.scene.add(stripeA);

    const stripeB = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.003, 4.2), stripeMat);
    stripeB.position.set(2.0, 0.001, -2);
    this.scene.add(stripeB);

    const stopLine = new THREE.Mesh(new THREE.BoxGeometry(3.9, 0.003, 0.08), stripeMat);
    stopLine.position.set(0, 0.001, 0.02);
    this.scene.add(stopLine);
  }

  private addGarageBackWall(): void {
    const wallMat = new THREE.MeshStandardMaterial({ color: 0x8f98a5, roughness: 0.88, metalness: 0.05 });
    const back = new THREE.Mesh(new THREE.BoxGeometry(16, 3.8, 0.16), wallMat);
    back.position.set(0, 1.65, -6.2);
    this.scene.add(back);

    const sideMat = new THREE.MeshStandardMaterial({ color: 0x7f8894, roughness: 0.9, metalness: 0.04 });
    const left = new THREE.Mesh(new THREE.BoxGeometry(0.16, 3.8, 9), sideMat);
    left.position.set(-5.2, 1.65, -2);
    this.scene.add(left);

    const right = new THREE.Mesh(new THREE.BoxGeometry(0.16, 3.8, 9), sideMat);
    right.position.set(5.2, 1.65, -2);
    this.scene.add(right);

    const pegboard = new THREE.Mesh(
      new THREE.BoxGeometry(4, 1.8, 0.05),
      new THREE.MeshStandardMaterial({ color: 0x5d6672, roughness: 0.95, metalness: 0.05 })
    );
    pegboard.position.set(-2.1, 1.7, -6.1);
    this.scene.add(pegboard);
  }

  private addWorkbench(): void {
    const frameMat = new THREE.MeshStandardMaterial({ color: 0x2b313a, roughness: 0.75, metalness: 0.35 });
    const topMat = new THREE.MeshStandardMaterial({ color: 0x9c7b5d, roughness: 0.84, metalness: 0.05 });
    const top = new THREE.Mesh(new THREE.BoxGeometry(3.6, 0.09, 0.95), topMat);
    top.position.set(0, 0.92, -5.55);
    this.scene.add(top);

    const legOffsets = [
      { x: -1.65, z: -5.9 },
      { x: 1.65, z: -5.9 },
      { x: -1.65, z: -5.2 },
      { x: 1.65, z: -5.2 },
    ];
    for (const off of legOffsets) {
      const leg = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.85, 0.1), frameMat);
      leg.position.set(off.x, 0.48, off.z);
      this.scene.add(leg);
    }

    const lamp = new THREE.Mesh(
      new THREE.CylinderGeometry(0.06, 0.06, 0.28, 10),
      new THREE.MeshStandardMaterial({ color: 0xc5ccd8, roughness: 0.28, metalness: 0.8 })
    );
    lamp.position.set(0.9, 1.12, -5.5);
    lamp.rotation.z = 0.25;
    this.scene.add(lamp);
  }

  private addShelfWall(): void {
    const frameMat = new THREE.MeshStandardMaterial({ color: 0x3b434f, roughness: 0.75, metalness: 0.28 });
    const shelfMat = new THREE.MeshStandardMaterial({ color: 0x6f7883, roughness: 0.7, metalness: 0.15 });

    const frame = new THREE.Mesh(new THREE.BoxGeometry(1.8, 2.5, 0.45), frameMat);
    frame.position.set(4.1, 1.05, -5.25);
    this.scene.add(frame);

    const shelfYs = [0.3, 0.85, 1.4, 1.95];
    for (const y of shelfYs) {
      const shelf = new THREE.Mesh(new THREE.BoxGeometry(1.64, 0.06, 0.38), shelfMat);
      shelf.position.set(4.1, y, -5.25);
      this.scene.add(shelf);
    }
  }

  private addToolChest(): void {
    const chest = new THREE.Mesh(
      new THREE.BoxGeometry(1.1, 1.0, 0.6),
      new THREE.MeshStandardMaterial({ color: 0xb04234, roughness: 0.56, metalness: 0.32 })
    );
    chest.position.set(-4.15, 0.3, -5.2);
    this.scene.add(chest);

    const top = new THREE.Mesh(
      new THREE.BoxGeometry(1.16, 0.08, 0.64),
      new THREE.MeshStandardMaterial({ color: 0x252b33, roughness: 0.6, metalness: 0.45 })
    );
    top.position.set(-4.15, 0.84, -5.2);
    this.scene.add(top);
  }

  private addTireStack(): void {
    const tireMat = new THREE.MeshStandardMaterial({ color: 0x111418, roughness: 0.92, metalness: 0.08 });
    for (let i = 0; i < 3; i += 1) {
      const tire = new THREE.Mesh(new THREE.CylinderGeometry(0.37, 0.37, 0.24, 24), tireMat);
      tire.position.set(3.35, 0.14 + i * 0.25, -1.0);
      tire.rotation.z = Math.PI / 2;
      this.scene.add(tire);
    }
  }

  private addStorageCrates(): void {
    const crateMat = new THREE.MeshStandardMaterial({ color: 0x5f6f7f, roughness: 0.8, metalness: 0.1 });

    const crateA = new THREE.Mesh(new THREE.BoxGeometry(0.75, 0.42, 0.55), crateMat);
    crateA.position.set(-3.5, 0.02, -1.15);
    this.scene.add(crateA);

    const crateB = new THREE.Mesh(new THREE.BoxGeometry(0.62, 0.34, 0.5), crateMat);
    crateB.position.set(-2.9, 0.02, -1.45);
    this.scene.add(crateB);
  }
}
