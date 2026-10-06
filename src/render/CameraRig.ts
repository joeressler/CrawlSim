import * as THREE from "three";
import type { Vec3 } from "../vehicles/types.ts";

function lerpAngle(from: number, to: number, t: number): number {
  // atan2 path wraps correctly for negative deltas (JS % does not).
  const delta = Math.atan2(Math.sin(to - from), Math.cos(to - from));
  return from + delta * t;
}

export class CameraRig {
  readonly camera: THREE.PerspectiveCamera;
  private readonly offset: THREE.Vector3;
  /** Smoothed heading yaw (rad). Camera sits behind the truck using this. */
  private laggedYaw = 0;
  private headingReady = false;
  /** Slight lag vs truck yaw — lower = more catch-up delay. */
  private readonly yawTween = 0.045;
  private readonly posTween = 0.08;
  private readonly desired = new THREE.Vector3();
  private readonly rotatedOffset = new THREE.Vector3();

  constructor(cameraOffset: Vec3) {
    this.camera = new THREE.PerspectiveCamera(60, innerWidth / innerHeight, 0.1, 200);
    this.offset = new THREE.Vector3(cameraOffset.x, cameraOffset.y, cameraOffset.z);
    this.camera.position.set(0, 5, 10);
  }

  /**
   * Follow chassis. `headingYaw` is truck yaw about world up; offset is rotated
   * by a lagged copy so the camera swings behind with smooth catch-up.
   */
  follow(target: THREE.Vector3, headingYaw = 0): void {
    if (!this.headingReady) {
      this.laggedYaw = headingYaw;
      this.headingReady = true;
    } else {
      this.laggedYaw = lerpAngle(this.laggedYaw, headingYaw, this.yawTween);
    }

    const cos = Math.cos(this.laggedYaw);
    const sin = Math.sin(this.laggedYaw);
    // Chassis forward is -Z; stock offset +Z sits behind at yaw 0.
    this.rotatedOffset.set(
      this.offset.x * cos + this.offset.z * sin,
      this.offset.y,
      -this.offset.x * sin + this.offset.z * cos
    );
    this.desired.set(
      target.x + this.rotatedOffset.x,
      target.y + this.rotatedOffset.y,
      target.z + this.rotatedOffset.z
    );
    this.camera.position.lerp(this.desired, this.posTween);
    this.camera.lookAt(target.x, target.y, target.z);
  }

  /** Instant place (scene switch / rebuild) — no lag catch-up frame. */
  snapTo(target: THREE.Vector3, headingYaw = 0): void {
    this.laggedYaw = headingYaw;
    this.headingReady = true;
    const cos = Math.cos(headingYaw);
    const sin = Math.sin(headingYaw);
    this.rotatedOffset.set(
      this.offset.x * cos + this.offset.z * sin,
      this.offset.y,
      -this.offset.x * sin + this.offset.z * cos
    );
    this.camera.position.set(
      target.x + this.rotatedOffset.x,
      target.y + this.rotatedOffset.y,
      target.z + this.rotatedOffset.z
    );
    this.camera.lookAt(target.x, target.y, target.z);
  }

  setOffset(offset: Vec3): void {
    this.offset.set(offset.x, offset.y, offset.z);
  }

  setFov(fov: number): void {
    if (Math.abs(this.camera.fov - fov) < 0.001) return;
    this.camera.fov = fov;
    this.camera.updateProjectionMatrix();
  }

  onResize(): void {
    this.camera.aspect = innerWidth / innerHeight;
    this.camera.updateProjectionMatrix();
  }
}