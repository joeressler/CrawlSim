/**
 * Solid-axle locate: one distance constraint per link (4-link + panhard).
 *
 * Rapier 0.21 has no rope joint. A 0.05 kg body between two sphericals chattered
 * against the chassis, so the rod is an XPBD-style impulse solved in preStep.
 * Heave and roll stay free; fore-aft, lateral, yaw, and wrap are held.
 * Do not stack these on spherical joints.
 */
import type RAPIER from "@dimforge/rapier3d-compat";
import * as THREE from "three";
import type { KitAxleRuntime, KitLinkRuntime } from "./kitBodies.ts";
import type { Vec3 } from "./types.ts";
import { clamp, invMassAlong } from "./impulse.ts";

const ITERATIONS = 16;
/** Fraction of each correction applied together (Jacobi). Stops redundant rods from spinning the axle. */
const RELAX = 0.35;
const BETA = 0.22;
/** Ignore tiny stretch so the bias does not buzz. */
const SLOP = 0.0006;
/** m/N. About 1 mm of stretch at 80 N, inside the 5 mm link gate. */
const COMPLIANCE = 1.2e-5;
const MAX_FORCE = 280;

const scratchQ = new THREE.Quaternion();
const scratchV = new THREE.Vector3();

function worldPoint(body: RAPIER.RigidBody, local: Vec3): { x: number; y: number; z: number } {
  const t = body.translation();
  const r = body.rotation();
  scratchQ.set(r.x, r.y, r.z, r.w);
  scratchV.set(local.x, local.y, local.z).applyQuaternion(scratchQ);
  return { x: t.x + scratchV.x, y: t.y + scratchV.y, z: t.z + scratchV.z };
}

export function applyDistanceLinks(
  chassis: RAPIER.RigidBody,
  axles: Map<string, KitAxleRuntime>,
  links: KitLinkRuntime[],
  dt: number
): void {
  if (!(dt > 0) || links.length === 0) return;
  const bodyByKey = new Map<string, RAPIER.RigidBody>();
  bodyByKey.set("chassis", chassis);
  for (const [id, axle] of axles) bodyByKey.set(id, axle.body);

  const maxJ = MAX_FORCE * dt;
  const soften = COMPLIANCE / (dt * dt);

  type Fix = {
    bodyA: RAPIER.RigidBody;
    bodyB: RAPIER.RigidBody;
    p0: { x: number; y: number; z: number };
    p1: { x: number; y: number; z: number };
    nx: number;
    ny: number;
    nz: number;
    j: number;
  };

  for (let iter = 0; iter < ITERATIONS; iter += 1) {
    const fixes: Fix[] = [];
    for (const link of links) {
      const bodyA = bodyByKey.get(link.from.bodyKey);
      const bodyB = bodyByKey.get(link.to.bodyKey);
      if (!bodyA || !bodyB) continue;
      const p0 = worldPoint(bodyA, link.from.local);
      const p1 = worldPoint(bodyB, link.to.local);
      const dx = p1.x - p0.x;
      const dy = p1.y - p0.y;
      const dz = p1.z - p0.z;
      const dist = Math.hypot(dx, dy, dz);
      if (dist < 1e-6) continue;
      const nx = dx / dist;
      const ny = dy / dist;
      const nz = dz / dist;
      const v0 = bodyA.velocityAtPoint(p0);
      const v1 = bodyB.velocityAtPoint(p1);
      const vn = (v1.x - v0.x) * nx + (v1.y - v0.y) * ny + (v1.z - v0.z) * nz;
      const w0 = invMassAlong(bodyA, p0.x, p0.y, p0.z, nx, ny, nz);
      const w1 = invMassAlong(bodyB, p1.x, p1.y, p1.z, nx, ny, nz);
      const error = dist - link.length;
      // Cap the correction speed. Uncapped Baumgarte launches the axle once stretch exceeds a few cm.
      const bias =
        Math.abs(error) <= SLOP ? 0 : clamp((BETA * error) / dt, -2.4, 2.4);
      const j = clamp((-(vn + bias) / (w0 + w1 + soften)) * RELAX, -maxJ, maxJ);
      if (j === 0) continue;
      fixes.push({ bodyA, bodyB, p0, p1, nx, ny, nz, j });
    }
    for (const fix of fixes) {
      fix.bodyA.applyImpulseAtPoint(
        { x: -fix.nx * fix.j, y: -fix.ny * fix.j, z: -fix.nz * fix.j },
        fix.p0,
        true
      );
      fix.bodyB.applyImpulseAtPoint(
        { x: fix.nx * fix.j, y: fix.ny * fix.j, z: fix.nz * fix.j },
        fix.p1,
        true
      );
    }
  }

  // One nudge per axle from its worst rod, so five links cannot stack the correction.
  const worst = new Map<string, { axle: RAPIER.RigidBody; x: number; y: number; z: number; abs: number }>();
  for (const link of links) {
    const bodyA = bodyByKey.get(link.from.bodyKey);
    const bodyB = bodyByKey.get(link.to.bodyKey);
    if (!bodyA || !bodyB) continue;
    const axleKey = link.to.bodyKey === "chassis" ? link.from.bodyKey : link.to.bodyKey;
    const axleBody = link.to.bodyKey === "chassis" ? bodyA : bodyB;
    if (axleBody === chassis) continue;
    const p0 = worldPoint(bodyA, link.from.local);
    const p1 = worldPoint(bodyB, link.to.local);
    const dx = p1.x - p0.x;
    const dy = p1.y - p0.y;
    const dz = p1.z - p0.z;
    const dist = Math.hypot(dx, dy, dz);
    const error = dist - link.length;
    const abs = Math.abs(error);
    if (abs < 0.2 || dist < 1e-6) continue;
    const prev = worst.get(axleKey);
    if (prev && prev.abs >= abs) continue;
    const shift = clamp(error, -0.02, 0.02) * 0.5;
    const towardChassis = link.to.bodyKey === "chassis" ? 1 : -1;
    worst.set(axleKey, {
      axle: axleBody,
      x: (dx / dist) * shift * towardChassis,
      y: (dy / dist) * shift * towardChassis,
      z: (dz / dist) * shift * towardChassis,
      abs,
    });
  }
  for (const fix of worst.values()) {
    const t = fix.axle.translation();
    fix.axle.setTranslation({ x: t.x + fix.x, y: t.y + fix.y, z: t.z + fix.z }, true);
  }
}
