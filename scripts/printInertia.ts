import { createHarness, idle } from "./crawlHarness.ts";

async function main() {
  const h = await createHarness();
  idle(h, 30);
  const kit = h.vehicle.kitSuspension()!;
  for (const [id, a] of kit.axles) {
    console.log(id, "mass", a.body.mass(), "inertia", a.body.principalInertia());
  }
}
main();
