/**
 * Soft crawler carcass: a fan of radial springs on the axle, not a rigid wheel.
 *
 * Rays sit in the wheel plane (articulated with the housing, yawed by steer).
 * Each hit inside the unloaded radius pushes the axle along that ray. The
 * resultant walks the hub up a lip as the leading rays load first. Force is
 * compression-only with a softer rebound damper, so a ledge dissipates instead
 * of bouncing. The Rapier hub sphere is only the collapsed core.
 *
 * Later: a node-ring carcass (BeamNG / Rigs of Rods) if the ray fan is not enough.
 */
import RAPIER from "@dimforge/rapier3d-compat";
import { HUB_GROUPS } from "../physics/collisionGroups.ts";
import type { TireDef } from "./types.ts";

/** Tread samples from slightly aft of down to ~62° forward, then two sidewalls. */
export const TREAD_ANGLES = [-0.4, -0.15, 0.08, 0.3, 0.52, 0.78, 1.08] as const;
const SIDE_ANGLE = 0.5;
export const PATCH_RAYS = TREAD_ANGLES.length + 2;

/** Rays that share a flat contact. Full `radialK` would stack and pogo. */
const PATCH_SHARE = 3;
const SIDE_SHARE = 0.35;

export type TirePatchHit = {
  fn: number;
  /** Deepest filtered squash (m). */
  deflection: number;
  /** Unit direction from the ground toward the hub (world up on flat ground). */
  nx: number;
  ny: number;
  nz: number;
  px: number;
  py: number;
  pz: number;
};

const ray = new RAPIER.Ray({ x: 0, y: 0, z: 0 }, { x: 0, y: -1, z: 0 });

function normalize(x: number, y: number, z: number): { x: number; y: number; z: number } | null {
  const len = Math.hypot(x, y, z);
  if (len < 1e-6) return null;
  return { x: x / len, y: y / len, z: z / len };
}

/**
 * Cast the patch and apply radial impulses on `axle`. `filter` is per-ray
 * deflection state (length PATCH_RAYS), updated in place.
 */
export function applyTirePatch(
  world: RAPIER.World,
  axle: RAPIER.RigidBody,
  hubX: number,
  hubY: number,
  hubZ: number,
  axisX: number,
  axisY: number,
  axisZ: number,
  hintX: number,
  hintY: number,
  hintZ: number,
  radius: number,
  tire: TireDef,
  filter: number[],
  dt: number
): TirePatchHit {
  const empty: TirePatchHit = {
    fn: 0,
    deflection: 0,
    nx: 0,
    ny: 1,
    nz: 0,
    px: hubX,
    py: hubY - radius,
    pz: hubZ,
  };
  if (!(dt > 0) || !(radius > 0)) return empty;

  const gravity = world.gravity;
  const gMag = Math.hypot(gravity.x, gravity.y, gravity.z) || 9.81;
  let dx = gravity.x / gMag;
  let dy = gravity.y / gMag;
  let dz = gravity.z / gMag;
  const along = dx * axisX + dy * axisY + dz * axisZ;
  dx -= axisX * along;
  dy -= axisY * along;
  dz -= axisZ * along;
  const down = normalize(dx, dy, dz) ?? { x: gravity.x / gMag, y: gravity.y / gMag, z: gravity.z / gMag };

  let fx = axisY * down.z - axisZ * down.y;
  let fy = axisZ * down.x - axisX * down.z;
  let fz = axisX * down.y - axisY * down.x;
  const fwdN = normalize(fx, fy, fz);
  if (!fwdN) return empty;
  fx = fwdN.x;
  fy = fwdN.y;
  fz = fwdN.z;
  if (fx * hintX + fy * hintY + fz * hintZ < 0) {
    fx = -fx;
    fy = -fy;
    fz = -fz;
  }

  const maxSquash = tire.maxDeflection * radius;
  const blend = 1 - Math.exp(-dt / tire.deflectionFilter);
  const vel = axle.velocityAtPoint({ x: hubX, y: hubY, z: hubZ });

  let fn = 0;
  let deflection = 0;
  let nx = 0;
  let ny = 0;
  let nz = 0;
  let px = hubX;
  let py = hubY;
  let pz = hubZ;
  const pushes: { x: number; y: number; z: number; px: number; py: number; pz: number; mag: number }[] = [];

  const castOne = (index: number, dirX: number, dirY: number, dirZ: number, share: number): void => {
    const dir = normalize(dirX, dirY, dirZ);
    if (!dir) return;
    ray.origin.x = hubX;
    ray.origin.y = hubY;
    ray.origin.z = hubZ;
    ray.dir.x = dir.x;
    ray.dir.y = dir.y;
    ray.dir.z = dir.z;
    const hit = world.castRayAndGetNormal(
      ray,
      radius,
      true,
      undefined,
      HUB_GROUPS,
      undefined,
      axle
    );
    let raw = 0;
    let toi = radius;
    if (hit && hit.timeOfImpact <= radius) {
      toi = hit.timeOfImpact;
      raw = Math.min(Math.max(0, radius - toi), maxSquash);
    }
    const prev = filter[index] ?? 0;
    const filtered = Math.min(maxSquash, Math.max(0, prev + (raw - prev) * blend));
    filter[index] = filtered;
    if (filtered > deflection) {
      deflection = filtered;
      px = hubX + dir.x * toi;
      py = hubY + dir.y * toi;
      pz = hubZ + dir.z * toi;
    }
    const closing = vel.x * dir.x + vel.y * dir.y + vel.z * dir.z;
    const damper = (closing > 0 ? tire.radialC : tire.reboundC) * share;
    let dampForce = damper * (raw > 0 ? closing : 0);
    if (dampForce > 36) dampForce = 36;
    if (dampForce < -18) dampForce = -18;
    let mag = tire.radialK * share * filtered + dampForce;
    if (mag < 0 || filtered < 1e-5) mag = 0;
    if (mag <= 0) return;
    pushes.push({
      x: -dir.x,
      y: -dir.y,
      z: -dir.z,
      px: hubX + dir.x * toi,
      py: hubY + dir.y * toi,
      pz: hubZ + dir.z * toi,
      mag,
    });
    nx += -dir.x * filtered;
    ny += -dir.y * filtered;
    nz += -dir.z * filtered;
  };

  for (let i = 0; i < TREAD_ANGLES.length; i += 1) {
    const angle = TREAD_ANGLES[i]!;
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    castOne(i, down.x * c + fx * s, down.y * c + fy * s, down.z * c + fz * s, 1 / PATCH_SHARE);
  }
  const sideC = Math.cos(SIDE_ANGLE);
  const sideS = Math.sin(SIDE_ANGLE);
  castOne(
    TREAD_ANGLES.length,
    down.x * sideC + axisX * sideS,
    down.y * sideC + axisY * sideS,
    down.z * sideC + axisZ * sideS,
    SIDE_SHARE / PATCH_SHARE
  );
  castOne(
    TREAD_ANGLES.length + 1,
    down.x * sideC - axisX * sideS,
    down.y * sideC - axisY * sideS,
    down.z * sideC - axisZ * sideS,
    SIDE_SHARE / PATCH_SHARE
  );

  // One carcass budget so a fan of rays cannot stack into a launch.
  const WHEEL_FORCE_CAP = 70;
  let totalMag = 0;
  for (const push of pushes) totalMag += push.mag;
  const wheelScale = totalMag > WHEEL_FORCE_CAP ? WHEEL_FORCE_CAP / totalMag : 1;
  for (const push of pushes) {
    const mag = push.mag * wheelScale;
    fn += mag;
    const impulse = mag * dt;
    axle.applyImpulseAtPoint(
      { x: push.x * impulse, y: push.y * impulse, z: push.z * impulse },
      { x: push.px, y: push.py, z: push.pz },
      true
    );
  }

  const n = normalize(nx, ny, nz);
  return {
    fn,
    deflection,
    nx: n?.x ?? 0,
    ny: n?.y ?? 1,
    nz: n?.z ?? 0,
    px,
    py,
    pz,
  };
}
