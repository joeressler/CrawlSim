import type RAPIER from "@dimforge/rapier3d-compat";

export function clamp(value: number, min: number, max: number): number {
  if (!Number.isFinite(value)) return 0;
  if (max < min) return 0;
  return Math.min(max, Math.max(min, value));
}

/** 1/kg along a world direction at a point, including angular inertia. */
export function invMassAlong(
  body: RAPIER.RigidBody,
  px: number,
  py: number,
  pz: number,
  ux: number,
  uy: number,
  uz: number
): number {
  const com = body.worldCom();
  const rx = px - com.x;
  const ry = py - com.y;
  const rz = pz - com.z;
  const cx = ry * uz - rz * uy;
  const cy = rz * ux - rx * uz;
  const cz = rx * uy - ry * ux;
  const invI = body.effectiveWorldInvInertia();
  const e = invI.elements;
  const ix = e[0]! * cx + e[1]! * cy + e[2]! * cz;
  const iy = e[1]! * cx + e[3]! * cy + e[4]! * cz;
  const iz = e[2]! * cx + e[4]! * cy + e[5]! * cz;
  const inv = body.invMass() + cx * ix + cy * iy + cz * iz;
  return inv > 1e-8 ? inv : 1e-8;
}
