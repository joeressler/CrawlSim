import * as THREE from "three";
import { createHarness, idle, place, step } from "./crawlHarness.ts";
import { sampleSuspensionDiag } from "../src/vehicles/kitLinkDiagnostics.ts";

const h = await createHarness();
place(h, 0, 0.22, 3, 0);
idle(h, 120);

function pitch(ax: { body: { rotation: () => { x: number; y: number; z: number; w: number } } }) {
  const cr = h.vehicle.chassisBody.rotation();
  const ar = ax.body.rotation();
  return new THREE.Euler()
    .setFromQuaternion(
      new THREE.Quaternion(cr.x, cr.y, cr.z, cr.w)
        .invert()
        .multiply(new THREE.Quaternion(ar.x, ar.y, ar.z, ar.w)),
      "YXZ"
    ).x;
}

function loc(ax: { body: { translation: () => { x: number; y: number; z: number } } }) {
  const ct = h.vehicle.chassisBody.translation();
  const at = ax.body.translation();
  const cr = h.vehicle.chassisBody.rotation();
  return new THREE.Vector3(at.x - ct.x, at.y - ct.y, at.z - ct.z).applyQuaternion(
    new THREE.Quaternion(cr.x, cr.y, cr.z, cr.w).invert()
  );
}

const kit = h.vehicle.kitSuspension()!;
const front = kit.axles.get("front")!;
const rows = [];
for (let i = 0; i < 50; i++) {
  step(h, { throttle: 1, steer: 0, reset: false });
  const d = sampleSuspensionDiag(kit);
  const f = loc(front);
  const p = pitch(front);
  if (i % 5 === 0 || Math.abs(p) > 0.15 || f.z > -0.1) {
    rows.push({
      i,
      fZ: +f.z.toFixed(4),
      fY: +f.y.toFixed(4),
      pitch: +p.toFixed(3),
      link: +d.maxLinkAbsError.toFixed(5),
      vz: +h.vehicle.chassisBody.linvel().z.toFixed(3),
    });
  }
}
console.log(JSON.stringify(rows, null, 2));
