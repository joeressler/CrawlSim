/**
 * Deterministic fold-under diagnosis (no browser).
 *
 * Modes isolate gravity / coilovers / hub-drive / soft-planar.
 * Classifies A–F from Rapier axle + link measurements (not meshes).
 *
 *   npx tsx scripts/foldDiag.ts
 */
import * as THREE from "three";
import { createHarness, idle, place, step, type Harness } from "./crawlHarness.ts";
import { sampleSuspensionDiag } from "../src/vehicles/kitLinkDiagnostics.ts";
import type { KitAxleRuntime } from "../src/vehicles/kitBodies.ts";
import { kitDiagFlags, resetKitDiagFlags } from "../src/vehicles/kitDiagFlags.ts";

const DT = 1 / 60;
/** Front rest local Z ≈ -0.1565; fold-under = tuck toward / past belly. */
const FRONT_FOLD_Z = -0.05;
const REAR_FOLD_Z = 0.05;
const PITCH_EXCESS = 0.45;
const LINK_STRETCH = 0.008;

type Mode = "gravity" | "coils" | "drive" | "full" | "fullNoPlanar";

type FrameSnap = {
  step: number;
  mode: Mode;
  frontLocal: { x: number; y: number; z: number };
  rearLocal: { x: number; y: number; z: number };
  frontPitch: number;
  rearPitch: number;
  frontYaw: number;
  rearYaw: number;
  maxLinkAbs: number;
  maxLinkRel: number;
  chassisY: number;
};

function axleLocal(
  chassis: { translation: () => { x: number; y: number; z: number }; rotation: () => { x: number; y: number; z: number; w: number } },
  axle: KitAxleRuntime
): { x: number; y: number; z: number } {
  const ct = chassis.translation();
  const at = axle.body.translation();
  const cr = chassis.rotation();
  const inv = new THREE.Quaternion(cr.x, cr.y, cr.z, cr.w).invert();
  return new THREE.Vector3(at.x - ct.x, at.y - ct.y, at.z - ct.z).applyQuaternion(inv);
}

function axlePitchYaw(
  chassis: { rotation: () => { x: number; y: number; z: number; w: number } },
  axle: KitAxleRuntime
): { pitch: number; yaw: number } {
  const cr = chassis.rotation();
  const ar = axle.body.rotation();
  const cq = new THREE.Quaternion(cr.x, cr.y, cr.z, cr.w);
  const aq = new THREE.Quaternion(ar.x, ar.y, ar.z, ar.w);
  const e = new THREE.Euler().setFromQuaternion(cq.clone().invert().multiply(aq), "YXZ");
  return { pitch: e.x, yaw: e.y };
}

function snap(h: Harness, stepIndex: number, mode: Mode): FrameSnap | null {
  const kit = h.vehicle.kitSuspension();
  if (!kit) return null;
  const front = kit.axles.get("front")!;
  const rear = kit.axles.get("rear")!;
  const fl = axleLocal(h.vehicle.chassisBody, front);
  const rl = axleLocal(h.vehicle.chassisBody, rear);
  const fp = axlePitchYaw(h.vehicle.chassisBody, front);
  const rp = axlePitchYaw(h.vehicle.chassisBody, rear);
  const diag = sampleSuspensionDiag(kit);
  return {
    step: stepIndex,
    mode,
    frontLocal: fl,
    rearLocal: rl,
    frontPitch: fp.pitch,
    rearPitch: rp.pitch,
    frontYaw: fp.yaw,
    rearYaw: rp.yaw,
    maxLinkAbs: diag.maxLinkAbsError,
    maxLinkRel: diag.maxLinkRelError,
    chassisY: h.vehicle.chassisBody.translation().y,
  };
}

type ClassHit = {
  letter: "A" | "B" | "C" | "D" | "E" | "F";
  step: number;
  detail: string;
  frame: FrameSnap;
};

function classify(frame: FrameSnap, prev: FrameSnap | null): ClassHit | null {
  const pitchHit =
    Math.abs(frame.frontPitch) > PITCH_EXCESS || Math.abs(frame.rearPitch) > PITCH_EXCESS;
  const frontFold = frame.frontLocal.z > FRONT_FOLD_Z;
  const rearFold = frame.rearLocal.z < REAR_FOLD_Z;
  const stretch = frame.maxLinkAbs > LINK_STRETCH;
  const fell = frame.chassisY < -0.5;

  if (fell) {
    return { letter: "F", step: frame.step, detail: `chassisY=${frame.chassisY.toFixed(3)}`, frame };
  }
  if (stretch && (frontFold || rearFold || pitchHit)) {
    return {
      letter: "C",
      step: frame.step,
      detail: `maxLinkAbs=${frame.maxLinkAbs.toFixed(5)} with fold/pitch`,
      frame,
    };
  }
  if (pitchHit && (frontFold || rearFold)) {
    return {
      letter: "A",
      step: frame.step,
      detail: `pitch F/R=${frame.frontPitch.toFixed(3)}/${frame.rearPitch.toFixed(3)} z=${frame.frontLocal.z.toFixed(4)}/${frame.rearLocal.z.toFixed(4)}`,
      frame,
    };
  }
  if (frontFold || rearFold) {
    const letter = stretch ? "C" : Math.abs(frame.frontPitch) < 0.25 && Math.abs(frame.rearPitch) < 0.25 ? "B" : "A";
    // B = translate under with reasonable orientation; D if links OK (parallelogram permits)
    const finalLetter =
      letter === "B" && frame.maxLinkAbs <= LINK_STRETCH ? ("D" as const) : letter;
    return {
      letter: finalLetter === "B" ? "B" : finalLetter,
      step: frame.step,
      detail: `frontZ=${frame.frontLocal.z.toFixed(4)} rearZ=${frame.rearLocal.z.toFixed(4)} pitchF=${frame.frontPitch.toFixed(3)} linkAbs=${frame.maxLinkAbs.toFixed(5)}`,
      frame,
    };
  }
  void prev;
  return null;
}

type RunResult = {
  mode: Mode;
  frames: number;
  firstHit: ClassHit | null;
  end: FrameSnap | null;
  peakFrontZ: number;
  minRearZ: number;
  peakLinkAbs: number;
  peakPitch: number;
};

async function runMode(mode: Mode, frames: number, throttle: number): Promise<RunResult> {
  resetKitDiagFlags();
  if (mode === "gravity") {
    kitDiagFlags.skipCoilovers = true;
    kitDiagFlags.skipHubDrive = true;
  } else if (mode === "coils") {
    kitDiagFlags.skipHubDrive = true;
  } else if (mode === "drive") {
    kitDiagFlags.skipCoilovers = true;
  } else if (mode === "fullNoPlanar") {
    kitDiagFlags.skipSoftPlanar = true;
  }
  const h = await createHarness();
  place(h, 0, 0.22, 3, 0);
  idle(h, 120);

  let prev: FrameSnap | null = null;
  let firstHit: ClassHit | null = null;
  let peakFrontZ = -999;
  let minRearZ = 999;
  let peakLinkAbs = 0;
  let peakPitch = 0;
  let end: FrameSnap | null = null;

  for (let i = 0; i < frames; i += 1) {
    const th = mode === "gravity" || mode === "coils" ? 0 : throttle;
    step(h, { throttle: th, steer: 0, reset: false }, DT);
    const s = snap(h, i, mode);
    if (!s) break;
    end = s;
    peakFrontZ = Math.max(peakFrontZ, s.frontLocal.z);
    minRearZ = Math.min(minRearZ, s.rearLocal.z);
    peakLinkAbs = Math.max(peakLinkAbs, s.maxLinkAbs);
    peakPitch = Math.max(peakPitch, Math.abs(s.frontPitch), Math.abs(s.rearPitch));
    if (!firstHit) {
      const hit = classify(s, prev);
      if (hit) firstHit = hit;
    }
    prev = s;
  }

  resetKitDiagFlags();
  return {
    mode,
    frames,
    firstHit,
    end,
    peakFrontZ,
    minRearZ,
    peakLinkAbs,
    peakPitch,
  };
}

/** Geometry audit at rest: upper/lower parallelism and lengths. */
async function geometryAudit(): Promise<Record<string, unknown>> {
  const h = await createHarness();
  place(h, 0, 0.22, 3, 0);
  idle(h, 60);
  const kit = h.vehicle.kitSuspension()!;
  const diag = sampleSuspensionDiag(kit);
  const front = kit.axles.get("front")!;
  const links = kit.links.filter(
    (l) => l.from.bodyKey === "front" || l.to.bodyKey === "front" || l.from.bodyKey === "chassis"
  );
  const frontArms = kit.links.filter((l) => {
    const toFront = l.to.bodyKey === "front" || l.from.bodyKey === "front";
    return toFront && l.def.kind !== "panhard";
  });
  const lengths = frontArms.map((l) => ({
    id: l.id,
    length: l.length,
    from: l.from,
    to: l.to,
  }));
  // Chassis mount Z for upper vs lower (from stock via rest mounts on link defs)
  const chassisMountZs = frontArms.map((l) => {
    const chassisEnd = l.from.bodyKey === "chassis" ? l.from : l.to;
    return { id: l.id, chassisLocal: chassisEnd.local, axleLocal: (l.from.bodyKey === "front" ? l.from : l.to).local };
  });
  return {
    restFrontLocal: axleLocal(h.vehicle.chassisBody, front),
    restDiag: {
      maxLinkAbs: diag.maxLinkAbsError,
      maxLinkRel: diag.maxLinkRelError,
      axles: diag.axles,
    },
    frontArmLengths: lengths,
    frontArmMounts: chassisMountZs,
    note:
      "If upper/lower share chassis Z and axle Z with equal lengths → parallelogram (permits fore-aft arc fold).",
  };
}

async function reverseFoldProbe(): Promise<RunResult> {
  const h = await createHarness();
  place(h, 0, 0.22, 3, 0);
  idle(h, 120);
  let prev: FrameSnap | null = null;
  let firstHit: ClassHit | null = null;
  let peakFrontZ = -999;
  let minRearZ = 999;
  let peakLinkAbs = 0;
  let peakPitch = 0;
  let end: FrameSnap | null = null;
  for (let i = 0; i < 280; i += 1) {
    step(h, { throttle: -1, steer: 0, reset: false }, DT);
    const s = snap(h, i, "drive");
    if (!s) break;
    end = s;
    peakFrontZ = Math.max(peakFrontZ, s.frontLocal.z);
    minRearZ = Math.min(minRearZ, s.rearLocal.z);
    peakLinkAbs = Math.max(peakLinkAbs, s.maxLinkAbs);
    peakPitch = Math.max(peakPitch, Math.abs(s.frontPitch), Math.abs(s.rearPitch));
    if (!firstHit) {
      const hit = classify(s, prev);
      if (hit) firstHit = hit;
    }
    prev = s;
  }
  return {
    mode: "drive",
    frames: 280,
    firstHit,
    end,
    peakFrontZ,
    minRearZ,
    peakLinkAbs,
    peakPitch,
  };
}

async function main(): Promise<void> {
  const geo = await geometryAudit();
  const gravity = await runMode("gravity", 300, 0);
  const coils = await runMode("coils", 300, 0);
  const driveOnly = await runMode("drive", 200, 1);
  const full = await runMode("full", 480, 1);
  const fullNoPlanar = await runMode("fullNoPlanar", 480, 1);
  const reverse = await reverseFoldProbe();

  const report = {
    geometry: geo,
    thresholds: { FRONT_FOLD_Z, REAR_FOLD_Z, PITCH_EXCESS, LINK_STRETCH },
    gravity,
    coils,
    driveOnly,
    full,
    fullNoPlanar,
    reverse,
  };
  console.log(JSON.stringify(report, null, 2));
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
