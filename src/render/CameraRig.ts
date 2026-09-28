import * as THREE from "three";
import type { Vec3 } from "../vehicles/types.ts";

export class CameraRig {
  readonly camera: THREE.PerspectiveCamera;
  private readonly offset: THREE.Vector3;

  constructor(cameraOffset: Vec3) {
    this.camera = new THREE.PerspectiveCamera(60, innerWidth / innerHeight, 0.1, 200);
    this.offset = new THREE.Vector3(cameraOffset.x, cameraOffset.y, cameraOffset.z);
    this.camera.position.set(0, 5, 10);
  }

  follow(target: THREE.Vector3): void {
    const desired = new THREE.Vector3(
      target.x + this.offset.x,
      target.y + this.offset.y,
      target.z + this.offset.z
    );
    this.camera.position.lerp(desired, 0.08);
    this.camera.lookAt(target.x, target.y, target.z);
  }

  onResize(): void {
    this.camera.aspect = innerWidth / innerHeight;
    this.camera.updateProjectionMatrix();
  }
}
