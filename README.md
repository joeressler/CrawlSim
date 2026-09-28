# CrawlSim

RC rock-crawler vertical slice: box chassis, four wheels, ground + ramp, WASD drive, R reset.

## Scripts

- `npm run dev` â€” Vite dev server
- `npm run build` â€” typecheck and production bundle
- `npm run check` â€” `tsc --noEmit`
- `npm run preview` â€” serve the production build

## Controls

- **W / S** or arrows â€” throttle
- **A / D** or arrows â€” steer the front tires
- **R** â€” reset pose and velocities from the rig spawn

The ramp is straight ahead of spawn. A box ledge about one tire radius tall sits off to the right.

## Adding a rig

Handling, size, wheel layout, spawn, and camera follow offset live in JSON.

1. Copy [`src/data/rigs/stock.json`](src/data/rigs/stock.json).
2. Match the `RawRigJson` / `RigDef` shapes in [`src/vehicles/types.ts`](src/vehicles/types.ts).
3. Prefer the Strategy B **kit BOM** (`kit.chassis`, `axles`, `links`, `shocks`, `wheels`, `drive`). [`loadRig.ts`](src/data/loadRig.ts) validates mounts and fills legacy `chassis` / `wheels[].offset` / `suspension` so the existing chassis-ray drive path stays unchanged.
4. Legacy-only JSON (no `kit`) still loads if it already has `chassis`, `wheels`, and `suspension`.
5. Point the loader in [`src/data/loadRig.ts`](src/data/loadRig.ts) at the new file (or add a switch later). No game-loop changes.

### Kit BOM fields (Strategy B)

Top-level: `parts`, `spawn`, `maxSpeed`, `cameraOffset`, `kit`.

| Field | Role |
| --- | --- |
| `parts` | String ids for chassis / tires / motor / battery / axles / links / shocks (garage swap later) |
| `kit.chassis.halfExtents` | Cuboid half-size (m) |
| `kit.chassis.mass` | Chassis mass (kg) |
| `kit.chassis.mounts[]` | Named hardpoints on the chassis (`id`, local `offset`) |
| `kit.axles[]` | Solid axles: `id`, rest `offset` in chassis frame, local `mounts[]` |
| `kit.links[]` | Control arms: `id`, `from` / `to` as `{ part, mount }` (`part` is `"chassis"` or an axle id) |
| `kit.shocks[]` | Coilovers: `from` / `to` mounts plus `restLength`, `springK`, `damperC`, `maxTravel` |
| `kit.wheels[]` | Hubs on an axle: `axle`, `hubOffset` (axle-local), `radius`, `width`, `driven`, `steered` |
| `kit.drive` | Shared grip / motor caps: `mu`, `driveTorque`, `maxForce`, `steerAngle`, `maxAccel`, `minNormalY`, `minUpright` |

Phase 1 was data + validation only. **Phase 2a** added Strategy B articulation (axles + spherical links). **Phase 3** (SCX10.1-scale, 313 mm WB): equal-length parallel 4-link + panhard locate each axle; soft hub spheres are the only ground plant; vertical coilovers support ride height; hub drive restores throttle/steer. No chassis-ray spring (no double plant). **Phase 4**: Coulomb/ray grip retargeted to axle hubs (Fn estimate only; spheres plant; reverse steer yaw). WASD + reverse. No multipart chassis visual. **Phase 5**: procedural multipart SCX10.1 visuals (C-channel rails, shock hoops, skid, radio box, battery tray, bumpers; axle/link/shock primitives). Rapier proxies unchanged. **Phase 6**: suspension stability — lower chassis COM, gentler hub accel, crawl upright floors raised. Soft kit locate assist: WB spring F/R, axle yaw limits, skid transfer+shaft visuals with lateral-only stiffeners (no lock joints). Climb: hub long Coulomb along contact tangent + front lip face probe; kit reset copies chassis quat to axles. Bump/accel crumple: chassis↔axle collision pads (rest gap, CCD) + shock-axis bump packer — no COM hang-floor impulses (those fought links and made throttle axle-drag worse). Pitch restore signed by nose attitude. Stairs ~1–2.5 tire diameters. Strategy B / one plant / SCX10.1 unchanged.

### Legacy suspension and grip fields

Filled automatically from the kit (or authored directly in legacy JSON). Shared block `suspension` (override any of `restLength`, `springK`, `damperC`, `mu` on a single wheel):

| Field | Role |
| --- | --- |
| `restLength` | Unloaded ray length from the hardpoint down to the contact (m) |
| `springK` | Spring rate along the contact normal, `F = k * (restLength - dist)` (N/m) |
| `damperC` | Damper, `F = -c * closingSpeed` (NÂ·s/m). No force when the tire is unloaded |
| `mu` | Coulomb friction coefficient. `F_max = Î¼ * max(F_n, 0)` |
| `driveTorque` | Motor torque stub (NÂ·m). Caps longitudinal force at `driveTorque / wheel.radius` |
| `maxForce` | Extra cap on the friction vector (N) |
| `steerAngle` | Front steer limit (radians) |
| `maxAccel` | How fast throttle can change speed (m/sÂ²). Stops launch wheelies and reverse flips |
| `minNormalY` | Lowest world-up component of a contact normal that still supports the tire. Steeper faces are ignored so a ledge lip does not catch |
| `maxTravel` | Max strut compression from `restLength` (m). Tire cannot extend past restLength or compress past restLength âˆ’ maxTravel |
| `minUpright` | Chassis-up Â· world-up below this turns off tire support and drive so the rig cannot crawl upside down |

Per wheel (legacy): `offset` (hardpoint on the chassis), `radius`, `width`, `driven`, `steered`. `maxSpeed` clamps planar speed. Chassis `halfExtents.y` and hardpoint height set belly clearance so the cuboid does not catch a ledge before the tires crest it. Tire meshes are visual only â€” they have no colliders.

## Adding a part later

`PartIds` on `RigDef` (`chassis`, `tires`, `motor`, `battery`, optional `axles` / `links` / `shocks`) are string ids only in this slice. A garage can swap those ids and rebuild `CrawlerVehicle` from a new `RigDef` without rewriting `Game`.

Crawl forces live in [`src/vehicles/drive.ts`](src/vehicles/drive.ts): one suspension ray and a friction clamp per tire. Kit mounts/links/shocks are the BOM for later axle bodies and joints; the chassis-ray path is still what drives today.
## Headless tests (no browser)

Autonomous crawler gates for CI / unattended runs. Fixed dt = 1/60, scripted WASD-equivalent input.

| Command | What it asserts |
| --- | --- |
| `npm run settle` | Idle upright/hang/plant, forward smoke, steer + reverse-steer yaw |
| `npm run crawl-test` | Idle (via settle), flat forward displacement, ramp climb progress, ledge crest *attempt*, speed explode cap, upright tip floors |
| `npm run test` | `settle` then `crawl-test` (nonzero exit on any failure) |

`crawl-test` thresholds (see `scripts/crawlTest.ts`):

| Gate | Threshold |
| --- | --- |
| MAX_SPEED (no explode) | 8 m/s |
| Idle upright | >= 0.75 |
| Crawl min upright | >= 0.28 (final >= 0.30; not inverted) |
| Flat peak forward | >= 0.08 m |
| Ramp peak forward (toward ramp) | >= 0.08 m |
| Ramp from flat climbY + hang | climbY >= 0.35 m, minHang >= 8 mm |
| Stairs climbY + hang | climbY >= 0.12 m, minHang >= 8 mm |
| Throttle hang (F+R) | minHang >= 8 mm while driving |
| Ledge peak +X or maxY | >= 0.25 m or >= 0.12 m (crest attempt) |

Shared harness: `scripts/crawlHarness.ts`.

