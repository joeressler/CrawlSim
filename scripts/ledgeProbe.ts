import { createHarness, place, step, uprightY, speed } from "./crawlHarness.ts";

/** Match crawl-test ledge_crest scenario. */
const h = await createHarness();
place(h, 1.8, 0.35, 2.0, -Math.PI / 2);
idle: for (let i = 0; i < 120; i += 1) {
  step(h, { throttle: 0, steer: 0, reset: false });
}
const pre = uprightY(h.vehicle);
console.log("pre", pre.toFixed(3), "y", h.vehicle.chassisBody.translation().y.toFixed(3));
const x0 = h.vehicle.chassisBody.translation().x;
let peak = 0;
let maxY = h.vehicle.chassisBody.translation().y;
let minUp = 1;
for (let i = 0; i < 200; i += 1) {
  step(h, { throttle: 1, steer: 0, reset: false });
  const t = h.vehicle.chassisBody.translation();
  peak = Math.max(peak, t.x - x0);
  maxY = Math.max(maxY, t.y);
  minUp = Math.min(minUp, uprightY(h.vehicle));
  if (i % 40 === 39 || uprightY(h.vehicle) < 0.5) {
    console.log({
      i,
      dx: (t.x - x0).toFixed(3),
      y: t.y.toFixed(3),
      up: uprightY(h.vehicle).toFixed(3),
      spd: speed(h.vehicle).toFixed(3),
    });
  }
  if (uprightY(h.vehicle) < 0.2) break;
}
console.log({
  peak: peak.toFixed(3),
  maxY: maxY.toFixed(3),
  minUp: minUp.toFixed(3),
  finalUp: uprightY(h.vehicle).toFixed(3),
});
