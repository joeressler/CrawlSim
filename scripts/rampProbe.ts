import { createHarness, idle, place, step, speed, uprightY } from "./crawlHarness.ts";

const h = await createHarness();
place(h, 0, 1.5, -1.5, 0);
idle(h, 90);
const y0 = h.vehicle.chassisBody.translation().y;
const z0 = h.vehicle.chassisBody.translation().z;
console.log("pre", {
  y: y0,
  z: z0,
  upright: uprightY(h.vehicle),
  spd: speed(h.vehicle),
});
for (let i = 0; i < 120; i++) {
  step(h, { throttle: 1, steer: 0, reset: false });
  if (i % 20 === 0 || i === 119) {
    const t = h.vehicle.chassisBody.translation();
    console.log({
      i,
      dy: t.y - y0,
      dz: z0 - t.z,
      upright: uprightY(h.vehicle),
      spd: speed(h.vehicle),
      vz: h.vehicle.chassisBody.linvel().z,
    });
  }
}
