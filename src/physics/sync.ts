import type RAPIER from "@dimforge/rapier3d-compat";
import type { Object3D } from "three";

export function syncRigidBodyToObject(body: RAPIER.RigidBody, object: Object3D): void {
  const t = body.translation();
  const r = body.rotation();
  object.position.set(t.x, t.y, t.z);
  object.quaternion.set(r.x, r.y, r.z, r.w);
}
