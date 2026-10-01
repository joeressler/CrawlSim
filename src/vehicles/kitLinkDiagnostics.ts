/**
 * Observational suspension diagnostics — never mutates bodies, joints, or forces.
 */
import type RAPIER from "@dimforge/rapier3d-compat";
import * as THREE from "three";
import type { KitAxleRuntime, KitSuspensionRuntime } from "./kitBodies.ts";
import type { Vec3 } from "./types.ts";

const scratchQ = new THREE.Quaternion();
const scratchV = new THREE.Vector3();

export type LinkDiag = {
  id: string;
  kind: "arm" | "panhard";
  restLength: number;
  currentLength: number;
  absError: number;
  relError: number;
  fromValid: boolean;
  toValid: boolean;
  jointFromValid: boolean;
  jointToValid: boolean;
  /** World distance between joint anchors as reported by Rapier (if joints stored). */
  anchorCoincidenceFrom: number;
  anchorCoincidenceTo: number;
};

export type AxleDiag = {
  id: string;
  local: Vec3;
  lateral: number;
  pitch: number;
  yaw: number;
  maxLinkAbsError: number;
};

export type SuspensionDiag = {
  links: LinkDiag[];
  axles: AxleDiag[];
  wheelbase: number;
  restWheelbase: number;
  maxLinkAbsError: number;
  maxLinkRelError: number;
};

function bodyWorldPoint(body: RAPIER.RigidBody, local: Vec3): Vec3 {
  const t = body.translation();
  const r = body.rotation();
  scratchQ.set(r.x, r.y, r.z, r.w);
  scratchV.set(local.x, local.y, local.z).applyQuaternion(scratchQ);
  return { x: t.x + scratchV.x, y: t.y + scratchV.y, z: t.z + scratchV.z };
}

function jointWorldAnchor(
  joint: RAPIER.ImpulseJoint | undefined,
  which: 1 | 2
): Vec3 | null {
  if (!joint || !joint.isValid()) return null;
  const a = which === 1 ? joint.anchor1() : joint.anchor2();
  const body = which === 1 ? joint.body1() : joint.body2();
  if (!body) return null;
  return bodyWorldPoint(body, { x: a.x, y: a.y, z: a.z });
}

function dist(a: Vec3, b: Vec3): number {
  return Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}

function axleLocalInChassis(
  chassis: RAPIER.RigidBody,
  axle: KitAxleRuntime
): Vec3 {
  const ct = chassis.translation();
  const at = axle.body.translation();
  const cr = chassis.rotation();
  scratchQ.set(cr.x, cr.y, cr.z, cr.w).invert();
  scratchV.set(at.x - ct.x, at.y - ct.y, at.z - ct.z).applyQuaternion(scratchQ);
  return { x: scratchV.x, y: scratchV.y, z: scratchV.z };
}

function relativeYawPitch(
  chassis: RAPIER.RigidBody,
  axle: KitAxleRuntime
): { yaw: number; pitch: number } {
  const cr = chassis.rotation();
  const ar = axle.body.rotation();
  const cq = new THREE.Quaternion(cr.x, cr.y, cr.z, cr.w);
  const aq = new THREE.Quaternion(ar.x, ar.y, ar.z, ar.w);
  const rel = cq.clone().invert().multiply(aq);
  const e = new THREE.Euler().setFromQuaternion(rel, "YXZ");
  return { yaw: e.y, pitch: e.x };
}

/** Snapshot link length / joint validity and per-axle pose relative to chassis. */
export function sampleSuspensionDiag(kit: KitSuspensionRuntime): SuspensionDiag {
  const bodyByKey = new Map<string, RAPIER.RigidBody>();
  bodyByKey.set("chassis", kit.chassis);
  for (const [id, a] of kit.axles) bodyByKey.set(id, a.body);

  const links: LinkDiag[] = [];
  let maxLinkAbsError = 0;
  let maxLinkRelError = 0;

  for (const link of kit.links) {
    const fromBody = bodyByKey.get(link.from.bodyKey);
    const toBody = bodyByKey.get(link.to.bodyKey);
    const fromValid = !!fromBody;
    const toValid = !!toBody;
    let currentLength = 0;
    if (fromBody && toBody) {
      const fw = bodyWorldPoint(fromBody, link.from.local);
      const tw = bodyWorldPoint(toBody, link.to.local);
      currentLength = dist(fw, tw);
    }
    const absError = Math.abs(currentLength - link.length);
    const relError = link.length > 1e-6 ? absError / link.length : 0;
    maxLinkAbsError = Math.max(maxLinkAbsError, absError);
    maxLinkRelError = Math.max(maxLinkRelError, relError);

    const jFrom = link.jointFrom;
    const jTo = link.jointTo;
    const jointFromValid = !!jFrom && jFrom.isValid();
    const jointToValid = !!jTo && jTo.isValid();

    // Coincidence: joint anchor world positions should match mount world points.
    let anchorCoincidenceFrom = Number.NaN;
    let anchorCoincidenceTo = Number.NaN;
    if (fromBody && jointFromValid && jFrom) {
      const mountW = bodyWorldPoint(fromBody, link.from.local);
      // jointFrom connects fromBody (anchor1) to link (anchor2)
      const a1 = jointWorldAnchor(jFrom, 1);
      const a2 = jointWorldAnchor(jFrom, 2);
      if (a1 && a2) {
        anchorCoincidenceFrom = Math.max(dist(a1, mountW), dist(a1, a2));
      }
    }
    if (toBody && jointToValid && jTo) {
      const mountW = bodyWorldPoint(toBody, link.to.local);
      const a1 = jointWorldAnchor(jTo, 1);
      const a2 = jointWorldAnchor(jTo, 2);
      if (a1 && a2) {
        // jointTo connects link (anchor1) to toBody (anchor2)
        anchorCoincidenceTo = Math.max(dist(a2, mountW), dist(a1, a2));
      }
    }

    links.push({
      id: link.id,
      kind: link.def.kind === "panhard" ? "panhard" : "arm",
      restLength: link.length,
      currentLength,
      absError,
      relError,
      fromValid,
      toValid,
      jointFromValid,
      jointToValid,
      anchorCoincidenceFrom,
      anchorCoincidenceTo,
    });
  }

  const front = kit.axles.get("front");
  const rear = kit.axles.get("rear");
  let wheelbase = 0;
  let restWheelbase = 0;
  if (front && rear) {
    const ft = front.body.translation();
    const rt = rear.body.translation();
    wheelbase = Math.hypot(ft.x - rt.x, ft.y - rt.y, ft.z - rt.z);
    restWheelbase = Math.hypot(
      front.restLocal.x - rear.restLocal.x,
      front.restLocal.y - rear.restLocal.y,
      front.restLocal.z - rear.restLocal.z
    );
  }

  const axles: AxleDiag[] = [];
  for (const axle of kit.axles.values()) {
    const local = axleLocalInChassis(kit.chassis, axle);
    const { yaw, pitch } = relativeYawPitch(kit.chassis, axle);
    let axleMax = 0;
    for (const ld of links) {
      const link = kit.links.find((l) => l.id === ld.id);
      if (!link) continue;
      if (link.from.bodyKey !== axle.id && link.to.bodyKey !== axle.id) continue;
      axleMax = Math.max(axleMax, ld.absError);
    }
    axles.push({
      id: axle.id,
      local,
      lateral: local.x,
      pitch,
      yaw,
      maxLinkAbsError: axleMax,
    });
  }

  return {
    links,
    axles,
    wheelbase,
    restWheelbase,
    maxLinkAbsError,
    maxLinkRelError,
  };
}

/** Max absolute link length error across all links (meters). */
export function maxLinkLengthError(kit: KitSuspensionRuntime): number {
  return sampleSuspensionDiag(kit).maxLinkAbsError;
}
