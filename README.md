# CrawlSim

RC rock-crawler vertical slice: box chassis, four wheels, ground + ramp, WASD drive, R reset.

## Scripts

- `npm run dev` — Vite dev server
- `npm run build` — typecheck and production bundle
- `npm run check` — `tsc --noEmit`
- `npm run preview` — serve the production build

## Controls

- **W / S** or arrows — throttle
- **A / D** or arrows — steer the front tires
- **R** — reset pose and velocities from the rig spawn

The ramp is straight ahead of spawn. A box ledge about one tire radius tall sits off to the right.

## Adding a rig

Handling, size, wheel layout, spawn, and camera follow offset live in JSON.

1. Copy [`src/data/rigs/stock.json`](src/data/rigs/stock.json).
2. Match the `RigDef` shape in [`src/vehicles/types.ts`](src/vehicles/types.ts).
3. Point the loader in [`src/data/loadRig.ts`](src/data/loadRig.ts) at the new file (or add a switch later). No game-loop changes.

### Suspension and grip fields

Shared block `suspension` (override any of `restLength`, `springK`, `damperC`, `mu` on a single wheel):

| Field | Role |
| --- | --- |
| `restLength` | Unloaded ray length from the hardpoint down to the contact (m) |
| `springK` | Spring rate along the contact normal, `F = k * (restLength - dist)` (N/m) |
| `damperC` | Damper, `F = -c * closingSpeed` (N·s/m). No force when the tire is unloaded |
| `mu` | Coulomb friction coefficient. `F_max = μ * max(F_n, 0)` |
| `driveTorque` | Motor torque stub (N·m). Caps longitudinal force at `driveTorque / wheel.radius` |
| `maxForce` | Extra cap on the friction vector (N) |
| `steerAngle` | Front steer limit (radians) |
| `maxAccel` | How fast throttle can change speed (m/s²). Stops launch wheelies and reverse flips |
| `minNormalY` | Lowest world-up component of a contact normal that still supports the tire. Steeper faces are ignored so a ledge lip does not catch |

Per wheel: `offset` (hardpoint on the chassis), `radius` (tire radius and rolling radius), `width`, `driven`, `steered`. `maxSpeed` clamps planar speed. Chassis `halfExtents.y` and hardpoint height set belly clearance so the cuboid does not catch a ledge before the tires crest it. Tire meshes are visual only — they have no colliders.

## Adding a part later

`PartIds` on `RigDef` (`chassis`, `tires`, `motor`, `battery`) are string ids only in this slice. A garage can swap those ids and rebuild `CrawlerVehicle` from a new `RigDef` without rewriting `Game`.

Crawl forces live in [`src/vehicles/drive.ts`](src/vehicles/drive.ts): one suspension ray and a friction clamp per tire. Next step is real wheel joints, then lockers.
