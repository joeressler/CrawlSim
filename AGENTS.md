You are setting up CrawlSim: a real, extensible RC rock-crawler game (not a one-file demo).

## Stack (fixed)
- Vite + TypeScript
- Vanilla Three.js (NO React / R3F)
- @dimforge/rapier3d-compat for physics
- Code-first: all positions/sizes in TS/JSON, no Blender, no editor scene files

## Goals
1. Replace any single-file demo with a modular game architecture.
2. First playable vertical slice: box chassis crawler, 4 wheels, ground + ramp, WASD drive, R reset, follow camera.
3. Architecture must support later: swappable parts (chassis/tires/motor/battery), multiple trails, garage UI, telemetry (voltage, tip-over).

## Hard rules
- Do NOT use React, Godot, or cannon-es.
- Do NOT dump everything in main.ts.
- Do NOT invent fancy 3D models; BoxGeometry / CylinderGeometry / primitives only.
- Prefer small pure modules + clear interfaces. Agents will extend this later.
- Keep `npm run dev` and `npm run build` working.
- Add a simple `npm run check` if useful (`tsc --noEmit`).

## Target folder layout (create/adapt as needed)
```
src/
    main.ts                 # boot only: create canvas, start Game
game/
    Game.ts               # owns loop, pause, reset trail
    Time.ts               # dt clamp
render/
    Renderer.ts           # WebGLRenderer, resize, render(scene, camera)
    CameraRig.ts          # follow / lookAt chassis
    lights.ts
physics/
    PhysicsWorld.ts       # RAPIER.init, World, step(dt)
    sync.ts               # copy rigid body → Object3D
input/
    Input.ts              # keyboard → { throttle, steer, reset }
vehicles/
    types.ts              # RigDef, PartIds, WheelDef
    CrawlerVehicle.ts     # build chassis + wheels from RigDef; applyDrive(input)
    drive.ts              # throttle/steer → forces (ray-contact aware)
world/
    TrailScene.ts         # ground, ramp, lights; spawn pose
data/
    rigs/stock.json       # mass, chassis, wheels, suspension (spring/damper/μ/torque), maxSpeed, cameraOffset
ui/
    Hud.ts                # optional: "WASD drive · R reset" text overlay
    styles.css
```

## RigDef (JSON) — must exist and be loaded
Include at least: chassis halfExtents, mass, spawn {x,y,z}, wheels[{id, offset, radius, width, driven, steered}], suspension {restLength, springK, damperC, mu, driveTorque, maxForce, steerAngle, maxAccel, minNormalY}, maxSpeed, cameraOffset.
CrawlerVehicle reads RigDef; changing JSON must change handling/size without rewriting game code.

## Vehicle physics (current)
- Dynamic cuboid chassis collider; fixed ground + ramp colliders
- Wheels: visual cylinders only (no solid wheel colliders for climbing); synced to per-wheel suspension length
- Drive: per-wheel suspension rays + spring/damper normals + Coulomb longitudinal/lateral grip in `vehicles/drive.ts` (not a single chassis world-forward impulse)
- Handling numbers live in `src/data/rigs/stock.json` (`RigDef.suspension` / per-wheel overrides)
- Reset restores translation/rotation/velocities from RigDef.spawn
- Later upgrade path still open: wheel joints / lockers / richer tire model

## Game loop
fixed pattern: poll input → vehicle.preStep → world.step → sync meshes → camera → render.
Clamp dt (e.g. max 1/30).

## main.ts
Only: import css, await init, `new Game(document.body).start()`.

## Deliverables
1. Implement the layout above (delete/refactor demo code).
2. stock.json + loader.
3. Short README.md: scripts, controls, how to add a part/rig later.
4. Keep drive comments listing next upgrades (joints / lockers / richer tires).
5. Ensure TypeScript is strict enough; no `any` unless unavoidable.

## Out of scope for this pass
Garage UI, multiple trails, battery model, networking, assets pipeline.

## Done when
- `npm run dev` shows crawler that drives up the ramp and R resets
- Architecture matches the modular layout
- Handling numbers come from `data/rigs/stock.json`
- README explains extension points

Start by inspecting the current Vite project, then implement module-by-module, wiring Game last. Do not leave a god-file main.ts.

## Learned User Preferences
- Prefer real crawl behavior: per-wheel suspension and grip so the rig can climb lips/ledges, not arcade slide or spin-in-place.
- Keep tire meshes visual-only; do not rely on solid wheel mesh colliders to climb.
- Do not drive with a single chassis impulse along world/forward.
- Steering should pull the chassis in reverse as well as forward (no reverse-only slide).

## Learned Workspace Facts
- Current drive model is per-wheel suspension rays plus Coulomb friction in `src/vehicles/drive.ts`; chassis keeps a cuboid collider.
- Suspension and grip tunables live in `src/data/rigs/stock.json` under `RigDef.suspension` (with optional per-wheel overrides).