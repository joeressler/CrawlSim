/**
 * Headless autonomous crawler suite (no browser).
 *
 *   npm run crawl-test
 *
 * Gates catch explode/orbit (|v|), tip-over (upright floor), and scripted
 * progress on flat / toward ramp / at ledge. Peak progress is used because
 * a long throttle can settle mid-run. Play-path gates (ramp from flat, stairs)
 * catch false-greens from teleport-on-ramp / impulse-only bump tests.
 */
import { runIdleSettle } from "./settleKit.ts";
import * as THREE from "three";
import {
  check,
  createHarness,
  idle,
  place,
  speed,
  step,
  uprightY,
  type GateFailure,
} from "./crawlHarness.ts";
import { sampleSuspensionDiag } from "../src/vehicles/kitLinkDiagnostics.ts";

const MAX_SPEED = 8;
const MIN_UPRIGHT_IDLE = 0.75;
/** Not inverted / rolling cage — long throttle currently settles ~0.35 pitch. */
const MIN_UPRIGHT_CRAWL = 0.35;
const MIN_UPRIGHT_FINAL = 0.35;

const MIN_FLAT_PEAK = 0.08;
const MIN_RAMP_PEAK = 0.08;
const MIN_LEDGE_PEAK = 0.25;
const MIN_LEDGE_Y = 0.12;
/** Planted on ramp surface — must gain height. */
const MIN_RAMP_CLIMB_Y = 0.25;
/** From flat spawn: real approach must crest onto the ramp. */
const MIN_RAMP_FROM_FLAT_CLIMB_Y = 0.28;
/** Stairs lane: must crest the second tread (~0.16m). */
const MIN_STAIRS_CLIMB_Y = 0.15;
/** Fixed-length link integrity (absolute meters / relative). */
const MAX_LINK_ABS_ERROR = 0.005;
const MAX_LINK_REL_ERROR = 0.05;
/** Panhard lateral locate bound (chassis-local |x| of either axle). */
const MAX_AXLE_LATERAL = 0.04;
/** Axle yaw vs chassis about up — ram must not 180 the axle. */
const MAX_AXLE_YAW_RAM = 0.9;
/** Chassis Y - front axle Y under bump/slam - must not crumple onto axle. */
const MIN_BUMP_FRONT_HANG = 0.008;
/** Play-path crumple floor (stairs / ramp lip) — hang must stay non-negative. */
const MIN_PLAY_HANG = 0.0;
/** Axle center must stay below rail underside in chassis frame (play sit-on-axle). */
const MAX_AXLE_LOCAL_Y = -0.020;
/** Max |axleLocal.z - rest.z| — soft links otherwise migrate axle past pads (~14cm). */
const MAX_AXLE_STATION_DRIFT = 0.16;
/** Front axle chassis-local Z — rest is -0.1565; lip tuck without soft anti-fold.
 * Crumple pads own hard stop; allow mild tuck past prior -0.025 assist gate. */
const MAX_FRONT_AXLE_LOCAL_Z = 0.005;
/** Rear axle chassis-local Z — rest is +0.1565; reverse fold tucks forward under belly.
 * Rear belly stop ~12cm forward of rest; gate matches held station. */
const MIN_REAR_AXLE_LOCAL_Z = 0.015;
/**
 * Flat-throttle fold-under: Rapier axle pitch vs chassis (rad).
 * Rest≈0; foldDiag pre-fix peak≈1.5. Scale: WB≈0.31m, useful travel≈±0.35 rad.
 */
/** Flat WOT axle pitch — climb-power Coulomb leaves ~0.5 rad peaks; hard fold is Z/link. */
const MAX_THROTTLE_AXLE_PITCH = 0.52;
/** Front local Z under same throttle — rest -0.1565; reject belly tuck past -0.08. */
const MAX_THROTTLE_FRONT_LOCAL_Z = -0.08;
/** Link length error under throttle (m) — solver soft stretch ceiling. */
const MAX_THROTTLE_LINK_ABS = 0.006;


function axleYawAbs(chassis: { rotation: () => { x: number; y: number; z: number; w: number } }, axle: { rotation: () => { x: number; y: number; z: number; w: number } }): number {
  const cr = chassis.rotation();
  const ar = axle.rotation();
  const cq = new THREE.Quaternion(cr.x, cr.y, cr.z, cr.w);
  const aq = new THREE.Quaternion(ar.x, ar.y, ar.z, ar.w);
  const up = new THREE.Vector3(0, 1, 0).applyQuaternion(cq);
  const chassisX = new THREE.Vector3(1, 0, 0).applyQuaternion(cq).projectOnPlane(up);
  const axleX = new THREE.Vector3(1, 0, 0).applyQuaternion(aq).projectOnPlane(up);
  if (chassisX.lengthSq() < 1e-8 || axleX.lengthSq() < 1e-8) return 0;
  chassisX.normalize();
  axleX.normalize();
  const cos = Math.min(1, Math.max(-1, axleX.dot(chassisX)));
  return Math.abs(Math.atan2(up.dot(new THREE.Vector3().crossVectors(chassisX, axleX)), cos));
}

/** Chassis-up separation chassisCOM - axleCOM (m). Positive = axle below rails. */
function chassisUpHang(
  chassis: { translation: () => { x: number; y: number; z: number }; rotation: () => { x: number; y: number; z: number; w: number } },
  axle: { translation: () => { x: number; y: number; z: number } }
): number {
  const cr = chassis.rotation();
  const cq = new THREE.Quaternion(cr.x, cr.y, cr.z, cr.w);
  const up = new THREE.Vector3(0, 1, 0).applyQuaternion(cq);
  const ct = chassis.translation();
  const at = axle.translation();
  return (ct.x - at.x) * up.x + (ct.y - at.y) * up.y + (ct.z - at.z) * up.z;
}

/** Axle center in chassis local frame. */
function axleInChassis(
  chassis: { translation: () => { x: number; y: number; z: number }; rotation: () => { x: number; y: number; z: number; w: number } },
  axle: { translation: () => { x: number; y: number; z: number } }
): THREE.Vector3 {
  const cr = chassis.rotation();
  const inv = new THREE.Quaternion(cr.x, cr.y, cr.z, cr.w).invert();
  const ct = chassis.translation();
  const at = axle.translation();
  return new THREE.Vector3(at.x - ct.x, at.y - ct.y, at.z - ct.z).applyQuaternion(inv);
}

type ScenarioReport = {
  name: string;
  ok: boolean;
  metrics: Record<string, number>;
  failures: GateFailure[];
};

async function scenarioIdleUpright(): Promise<ScenarioReport> {
  const failures: GateFailure[] = [];
  const m = await runIdleSettle(5);
  check(failures, "idle_upright", m.upright >= MIN_UPRIGHT_IDLE, `upright=${m.upright.toFixed(3)}`);
  check(failures, "idle_chassis_y", m.chassisY >= 0.07, `chassisY=${m.chassisY.toFixed(3)}`);
  check(failures, "idle_rel_hang", m.relHang >= 0.03, `relHang=${m.relHang.toFixed(3)}`);
  check(failures, "idle_drive", m.driveDeltaZ >= 0.08, `driveDeltaZ=${m.driveDeltaZ.toFixed(3)}`);
  return {
    name: "idle_upright",
    ok: failures.length === 0,
    metrics: {
      upright: m.upright,
      chassisY: m.chassisY,
      relHang: m.relHang,
      driveDeltaZ: m.driveDeltaZ,
      maxAbsVy: m.maxAbsVy,
    },
    failures,
  };
}

async function scenarioFlatForward(): Promise<ScenarioReport> {
  const failures: GateFailure[] = [];
  const h = await createHarness();
  idle(h, 300);
  const pre = uprightY(h.vehicle);
  check(failures, "flat_pre_upright", pre >= 0.7, `pre=${pre.toFixed(3)}`);
  const z0 = h.vehicle.chassisBody.translation().z;
  const y0 = h.vehicle.chassisBody.translation().y;
  let peak = 0;
  let maxSpeed = 0;
  let minUpright = 1;
  for (let i = 0; i < 200; i += 1) {
    step(h, { throttle: 1, steer: 0, reset: false });
    const t = h.vehicle.chassisBody.translation();
    peak = Math.max(peak, z0 - t.z);
    maxSpeed = Math.max(maxSpeed, speed(h.vehicle));
    minUpright = Math.min(minUpright, uprightY(h.vehicle));
  }
  const y1 = h.vehicle.chassisBody.translation().y;
  const upright = uprightY(h.vehicle);
  check(failures, "flat_peak", peak >= MIN_FLAT_PEAK, `peak=${peak.toFixed(3)} need>=${MIN_FLAT_PEAK}`);
  check(failures, "flat_upright_min", minUpright >= MIN_UPRIGHT_CRAWL, `minUpright=${minUpright.toFixed(3)}`);
  check(failures, "flat_upright_final", upright >= MIN_UPRIGHT_FINAL, `upright=${upright.toFixed(3)}`);
  check(failures, "flat_no_explode", maxSpeed <= MAX_SPEED, `maxSpeed=${maxSpeed.toFixed(3)}`);
  check(failures, "flat_no_dive", y1 > y0 - 0.15, `y0=${y0.toFixed(3)} y1=${y1.toFixed(3)}`);
  return {
    name: "flat_forward",
    ok: failures.length === 0,
    metrics: { peak, maxSpeed, minUpright, upright, y0, y1, pre },
    failures,
  };
}

/** Plant on ramp surface and climb uphill (-Z). Requires real height gain. */
async function scenarioRampClimb(): Promise<ScenarioReport> {
  const failures: GateFailure[] = [];
  const h = await createHarness();
  place(h, 0, 1.5, -1.5, 0);
  idle(h, 90);
  const pre = uprightY(h.vehicle);
  check(failures, "ramp_pre_upright", pre >= 0.55, `pre=${pre.toFixed(3)}`);
  const y0 = h.vehicle.chassisBody.translation().y;
  const z0 = h.vehicle.chassisBody.translation().z;
  let peak = 0;
  let maxY = y0;
  let maxSpeed = 0;
  let minUpright = 1;
  for (let i = 0; i < 360; i += 1) {
    step(h, { throttle: 1, steer: 0, reset: false });
    const t = h.vehicle.chassisBody.translation();
    peak = Math.max(peak, z0 - t.z);
    maxY = Math.max(maxY, t.y);
    maxSpeed = Math.max(maxSpeed, speed(h.vehicle));
    minUpright = Math.min(minUpright, uprightY(h.vehicle));
  }
  const upright = uprightY(h.vehicle);
  const climbY = maxY - y0;
  check(failures, "ramp_climb_y", climbY >= MIN_RAMP_CLIMB_Y, `climbY=${climbY.toFixed(3)} need>=${MIN_RAMP_CLIMB_Y}`);
  check(failures, "ramp_peak", peak >= MIN_RAMP_PEAK, `peak=${peak.toFixed(3)} need>=${MIN_RAMP_PEAK}`);
  check(failures, "ramp_upright_min", minUpright >= MIN_UPRIGHT_CRAWL, `minUpright=${minUpright.toFixed(3)}`);
  check(failures, "ramp_upright_final", upright >= 0.35, `upright=${upright.toFixed(3)}`);
  check(failures, "ramp_no_explode", maxSpeed <= MAX_SPEED, `maxSpeed=${maxSpeed.toFixed(3)}`);
  return {
    name: "ramp_climb",
    ok: failures.length === 0,
    metrics: { peak, climbY, maxY, y0, maxSpeed, minUpright, upright, z0, pre },
    failures,
  };
}

/**
 * Real play path: spawn on flat, W toward ramp. Must gain height AND keep hang
 * (teleport-on-ramp climb was false-green while approach crumpled).
 */
async function scenarioRampFromFlat(): Promise<ScenarioReport> {
  const failures: GateFailure[] = [];
  const h = await createHarness();
  place(h, 0, 0.22, 3, 0);
  idle(h, 300);
  const kit = h.vehicle.kitSuspension();
  check(failures, "rflat_has_kit", !!kit, "kit missing");
  if (!kit) {
    return { name: "ramp_from_flat", ok: false, metrics: {}, failures };
  }
  const front = kit.axles.get("front")!;
  const rear = kit.axles.get("rear")!;
  const y0 = h.vehicle.chassisBody.translation().y;
  const z0 = h.vehicle.chassisBody.translation().z;
  let maxY = y0;
  let minZ = z0;
  let maxSpeed = 0;
  let minUpright = 1;
  let minHang = 999;
  let maxLocalYF = -999;
  let maxLocalYR = -999;
  let maxLocalZF = -999;
  for (let i = 0; i < 480; i += 1) {
    step(h, { throttle: 1, steer: 0, reset: false });
    const chassis = h.vehicle.chassisBody;
    const t = chassis.translation();
    maxY = Math.max(maxY, t.y);
    minZ = Math.min(minZ, t.z);
    maxSpeed = Math.max(maxSpeed, speed(h.vehicle));
    minUpright = Math.min(minUpright, uprightY(h.vehicle));
    minHang = Math.min(
      minHang,
      chassisUpHang(chassis, front.body),
      chassisUpHang(chassis, rear.body)
    );
    const lf = axleInChassis(chassis, front.body);
    const lr = axleInChassis(chassis, rear.body);
    maxLocalYF = Math.max(maxLocalYF, lf.y);
    maxLocalYR = Math.max(maxLocalYR, lr.y);
    maxLocalZF = Math.max(maxLocalZF, lf.z);
  }
  const climbY = maxY - y0;
  const peak = z0 - minZ;
  check(
    failures,
    "rflat_climb_y",
    climbY >= MIN_RAMP_FROM_FLAT_CLIMB_Y,
    `climbY=${climbY.toFixed(3)} need>=${MIN_RAMP_FROM_FLAT_CLIMB_Y}`
  );
  check(failures, "rflat_peak", peak >= 1.0, `peak=${peak.toFixed(3)} need>=1.0`);
  check(
    failures,
    "rflat_hang",
    minHang >= MIN_BUMP_FRONT_HANG,
    `minHang=${minHang.toFixed(3)} need>=${MIN_BUMP_FRONT_HANG}`
  );
  // Play-shaped rail gate: flat throttle_hang stayed green while W into ramp
  // crumpled (HEAD maxLocalY F/R ~ -0.018/-0.019 past rail -0.020 at landing).
  // Clip failure: front axle folds aft under rails on ramp lip (not only vertical hang).
  check(
    failures,
    "rflat_fold_front",
    maxLocalZF <= MAX_FRONT_AXLE_LOCAL_Z,
    `maxLocalZF=${maxLocalZF.toFixed(4)} need<=${MAX_FRONT_AXLE_LOCAL_Z} (rest=-0.1565)`
  );
  check(
    failures,
    "rflat_rail_front",
    maxLocalYF <= MAX_AXLE_LOCAL_Y,
    `maxLocalYF=${maxLocalYF.toFixed(4)} need<=${MAX_AXLE_LOCAL_Y}`
  );
  check(
    failures,
    "rflat_rail_rear",
    maxLocalYR <= MAX_AXLE_LOCAL_Y,
    `maxLocalYR=${maxLocalYR.toFixed(4)} need<=${MAX_AXLE_LOCAL_Y}`
  );
  check(failures, "rflat_upright_min", minUpright >= 0.35, `minUpright=${minUpright.toFixed(3)}`);
  check(failures, "rflat_no_explode", maxSpeed <= MAX_SPEED, `maxSpeed=${maxSpeed.toFixed(3)}`);
  return {
    name: "ramp_from_flat",
    ok: failures.length === 0,
    metrics: { climbY, peak, maxY, y0, minZ, minHang, maxLocalYF, maxLocalYR, maxLocalZF, maxSpeed, minUpright },
    failures,
  };
}

async function scenarioStairsClimb(): Promise<ScenarioReport> {
  const failures: GateFailure[] = [];
  const h = await createHarness();
  place(h, -6.5, 0.22, 3.5, 0);
  idle(h, 90);
  const kit = h.vehicle.kitSuspension();
  check(failures, "stairs_has_kit", !!kit, "kit missing");
  if (!kit) {
    return { name: "stairs_climb", ok: false, metrics: {}, failures };
  }
  const front = kit.axles.get("front")!;
  const rear = kit.axles.get("rear")!;
  const y0 = h.vehicle.chassisBody.translation().y;
  let maxY = y0;
  let maxSpeed = 0;
  let minUpright = 1;
  let minHang = 999;
  for (let i = 0; i < 520; i += 1) {
    step(h, { throttle: 1, steer: 0, reset: false });
    const t = h.vehicle.chassisBody.translation();
    maxY = Math.max(maxY, t.y);
    maxSpeed = Math.max(maxSpeed, speed(h.vehicle));
    minUpright = Math.min(minUpright, uprightY(h.vehicle));
    minHang = Math.min(
      minHang,
      chassisUpHang(h.vehicle.chassisBody, front.body),
      chassisUpHang(h.vehicle.chassisBody, rear.body)
    );
  }
  const climbY = maxY - y0;
  check(
    failures,
    "stairs_climb_y",
    climbY >= MIN_STAIRS_CLIMB_Y,
    `climbY=${climbY.toFixed(3)} need>=${MIN_STAIRS_CLIMB_Y}`
  );
  check(
    failures,
    "stairs_hang",
    minHang >= MIN_BUMP_FRONT_HANG,
    `minHang=${minHang.toFixed(3)} need>=${MIN_BUMP_FRONT_HANG}`
  );
  check(failures, "stairs_upright_min", minUpright >= 0.28, `minUpright=${minUpright.toFixed(3)}`);
  check(failures, "stairs_no_explode", maxSpeed <= MAX_SPEED, `maxSpeed=${maxSpeed.toFixed(3)}`);
  return {
    name: "stairs_climb",
    ok: failures.length === 0,
    metrics: { climbY, maxY, y0, minHang, maxSpeed, minUpright },
    failures,
  };
}

/** Face +X into ledge; peak +X progress and/or Y lift = crest attempt. */
async function scenarioLedgeCrest(): Promise<ScenarioReport> {
  const failures: GateFailure[] = [];
  const h = await createHarness();
  place(h, 1.8, 0.35, 2.0, -Math.PI / 2);
  idle(h, 120);
  const pre = uprightY(h.vehicle);
  check(failures, "ledge_pre_upright", pre >= 0.55, `pre=${pre.toFixed(3)}`);
  const x0 = h.vehicle.chassisBody.translation().x;
  let peak = 0;
  let maxY = h.vehicle.chassisBody.translation().y;
  let maxSpeed = 0;
  let minUpright = 1;
  for (let i = 0; i < 200; i += 1) {
    step(h, { throttle: 1, steer: 0, reset: false });
    const t = h.vehicle.chassisBody.translation();
    peak = Math.max(peak, t.x - x0);
    maxY = Math.max(maxY, t.y);
    maxSpeed = Math.max(maxSpeed, speed(h.vehicle));
    minUpright = Math.min(minUpright, uprightY(h.vehicle));
  }
  const upright = uprightY(h.vehicle);
  const attempt = peak >= MIN_LEDGE_PEAK || maxY >= MIN_LEDGE_Y;
  check(
    failures,
    "ledge_attempt",
    attempt,
    `peak=${peak.toFixed(3)} maxY=${maxY.toFixed(3)} need peak>=${MIN_LEDGE_PEAK} or maxY>=${MIN_LEDGE_Y}`
  );
  check(failures, "ledge_upright_min", minUpright >= MIN_UPRIGHT_CRAWL, `minUpright=${minUpright.toFixed(3)}`);
  check(failures, "ledge_upright_final", upright >= MIN_UPRIGHT_FINAL, `upright=${upright.toFixed(3)}`);
  check(failures, "ledge_no_explode", maxSpeed <= MAX_SPEED, `maxSpeed=${maxSpeed.toFixed(3)}`);
  return {
    name: "ledge_crest",
    ok: failures.length === 0,
    metrics: { peak, maxY, maxSpeed, minUpright, upright, x0, pre },
    failures,
  };
}

/** Bump/ledge impact: axle hop + chassis dive must not fold rails onto axle. */
async function scenarioBumpHang(): Promise<ScenarioReport> {
  const failures: GateFailure[] = [];
  const h = await createHarness();
  const kit = h.vehicle.kitSuspension();
  check(failures, "bump_has_kit", !!kit, "kit missing");
  if (!kit) {
    return { name: "bump_hang", ok: false, metrics: {}, failures };
  }
  const front = kit.axles.get("front")!;
  const rear = kit.axles.get("rear")!;

  // 1) Axle hop: front axle kicked up like a sharp ledge under the hubs.
  place(h, 0, 0.22, 3, 0);
  idle(h, 90);
  const restHang = chassisUpHang(h.vehicle.chassisBody, front.body);
  front.body.applyImpulse({ x: 0, y: 2.5, z: 0 }, true);
  let minHopHang = restHang;
  let maxSpeed = 0;
  let minUpright = 1;
  for (let i = 0; i < 90; i += 1) {
    step(h, { throttle: 0, steer: 0, reset: false });
    minHopHang = Math.min(minHopHang, chassisUpHang(h.vehicle.chassisBody, front.body));
    maxSpeed = Math.max(maxSpeed, speed(h.vehicle));
    minUpright = Math.min(minUpright, uprightY(h.vehicle));
  }

  // 2) Chassis dive onto planted axles (fold from above).
  place(h, 0, 0.22, 3, 0);
  idle(h, 60);
  h.vehicle.chassisBody.applyImpulse({ x: 0, y: -2.8, z: 0 }, true);
  let minDiveHang = chassisUpHang(h.vehicle.chassisBody, front.body);
  for (let i = 0; i < 90; i += 1) {
    step(h, { throttle: 0, steer: 0, reset: false });
    minDiveHang = Math.min(
      minDiveHang,
      chassisUpHang(h.vehicle.chassisBody, front.body),
      chassisUpHang(h.vehicle.chassisBody, rear.body)
    );
    maxSpeed = Math.max(maxSpeed, speed(h.vehicle));
    minUpright = Math.min(minUpright, uprightY(h.vehicle));
  }

  // 3) Sustained stair approach (play path) — short slam was false-green.
  place(h, -6.5, 0.22, 3.5, 0);
  idle(h, 60);
  let minSlamHang = chassisUpHang(h.vehicle.chassisBody, front.body);
  for (let i = 0; i < 180; i += 1) {
    step(h, { throttle: 1, steer: 0, reset: false });
    minSlamHang = Math.min(minSlamHang, chassisUpHang(h.vehicle.chassisBody, front.body));
    maxSpeed = Math.max(maxSpeed, speed(h.vehicle));
    minUpright = Math.min(minUpright, uprightY(h.vehicle));
  }

  const minHang = Math.min(minHopHang, minDiveHang, minSlamHang);
  check(
    failures,
    "bump_front_hang",
    minHang >= MIN_BUMP_FRONT_HANG,
    `minHang=${minHang.toFixed(3)} (hop=${minHopHang.toFixed(3)} dive=${minDiveHang.toFixed(3)} slam=${minSlamHang.toFixed(3)}) need>=${MIN_BUMP_FRONT_HANG}`
  );
  check(failures, "bump_upright_min", minUpright >= 0.28, `minUpright=${minUpright.toFixed(3)}`);
  check(failures, "bump_no_explode", maxSpeed <= MAX_SPEED, `maxSpeed=${maxSpeed.toFixed(3)}`);
  return {
    name: "bump_hang",
    ok: failures.length === 0,
    metrics: { restHang, minHopHang, minDiveHang, minSlamHang, minHang, maxSpeed, minUpright },
    failures,
  };
}

/** Flat W throttle: hang + rail clearance + longitudinal station (play-shaped).
 * Prior hang-only gate was false-green: COM hang stayed >=19mm while rear axle rose
 * through the rails (localY=-0.019) and front axle drifted 14cm aft past pads.
 */
async function scenarioThrottleHang(): Promise<ScenarioReport> {
  const failures: GateFailure[] = [];
  const h = await createHarness();
  place(h, 0, 0.22, 3, 0);
  idle(h, 120);
  const kit = h.vehicle.kitSuspension();
  check(failures, "thr_has_kit", !!kit, "kit missing");
  if (!kit) {
    return { name: "throttle_hang", ok: false, metrics: {}, failures };
  }
  const front = kit.axles.get("front")!;
  const rear = kit.axles.get("rear")!;
  const restF = chassisUpHang(h.vehicle.chassisBody, front.body);
  const restR = chassisUpHang(h.vehicle.chassisBody, rear.body);
  const restFz = front.restLocal.z;
  const restRz = rear.restLocal.z;
  let minHangF = restF;
  let minHangR = restR;
  let maxLocalYF = -999;
  let maxLocalYR = -999;
  let maxDriftF = 0;
  let maxDriftR = 0;
  let maxSpeed = 0;
  let minUpright = 1;
  let peak = 0;
  const z0 = h.vehicle.chassisBody.translation().z;
  for (let i = 0; i < 240; i += 1) {
    step(h, { throttle: 1, steer: 0, reset: false });
    const chassis = h.vehicle.chassisBody;
    const t = chassis.translation();
    peak = Math.max(peak, z0 - t.z);
    minHangF = Math.min(minHangF, chassisUpHang(chassis, front.body));
    minHangR = Math.min(minHangR, chassisUpHang(chassis, rear.body));
    const lf = axleInChassis(chassis, front.body);
    const lr = axleInChassis(chassis, rear.body);
    maxLocalYF = Math.max(maxLocalYF, lf.y);
    maxLocalYR = Math.max(maxLocalYR, lr.y);
    maxDriftF = Math.max(maxDriftF, Math.abs(lf.z - restFz));
    maxDriftR = Math.max(maxDriftR, Math.abs(lr.z - restRz));
    maxSpeed = Math.max(maxSpeed, speed(h.vehicle));
    minUpright = Math.min(minUpright, uprightY(h.vehicle));
  }
  const minHang = Math.min(minHangF, minHangR);
  check(failures, "thr_drive", peak >= 0.08, `peak=${peak.toFixed(3)} need>=0.08`);
  check(
    failures,
    "thr_hang_front",
    minHangF >= MIN_BUMP_FRONT_HANG,
    `minHangF=${minHangF.toFixed(3)} need>=${MIN_BUMP_FRONT_HANG}`
  );
  check(
    failures,
    "thr_hang_rear",
    minHangR >= MIN_BUMP_FRONT_HANG,
    `minHangR=${minHangR.toFixed(3)} need>=${MIN_BUMP_FRONT_HANG}`
  );
  check(
    failures,
    "thr_rail_front",
    maxLocalYF <= MAX_AXLE_LOCAL_Y,
    `maxLocalYF=${maxLocalYF.toFixed(4)} need<=${MAX_AXLE_LOCAL_Y}`
  );
  check(
    failures,
    "thr_rail_rear",
    maxLocalYR <= MAX_AXLE_LOCAL_Y,
    `maxLocalYR=${maxLocalYR.toFixed(4)} need<=${MAX_AXLE_LOCAL_Y}`
  );
  check(
    failures,
    "thr_station_front",
    maxDriftF <= MAX_AXLE_STATION_DRIFT,
    `maxDriftF=${maxDriftF.toFixed(4)} need<=${MAX_AXLE_STATION_DRIFT}`
  );
  check(
    failures,
    "thr_station_rear",
    maxDriftR <= MAX_AXLE_STATION_DRIFT,
    `maxDriftR=${maxDriftR.toFixed(4)} need<=${MAX_AXLE_STATION_DRIFT}`
  );
  check(failures, "thr_upright_min", minUpright >= 0.45, `minUpright=${minUpright.toFixed(3)}`);
  check(failures, "thr_no_explode", maxSpeed <= MAX_SPEED, `maxSpeed=${maxSpeed.toFixed(3)}`);
  return {
    name: "throttle_hang",
    ok: failures.length === 0,
    metrics: {
      restF,
      restR,
      minHangF,
      minHangR,
      minHang,
      maxLocalYF,
      maxLocalYR,
      maxDriftF,
      maxDriftR,
      peak,
      maxSpeed,
      minUpright,
    },
    failures,
  };
}

async function scenarioAxleRam(): Promise<ScenarioReport> {
  const failures: GateFailure[] = [];
  const h = await createHarness();
  place(h, 2.8, 0.22, 2.0, -Math.PI / 2);
  idle(h, 60);
  const kit = h.vehicle.kitSuspension();
  check(failures, "ram_has_kit", !!kit, "kit missing");
  if (!kit) {
    return { name: "axle_ram", ok: false, metrics: {}, failures };
  }
  const front = kit.axles.get("front")!;
  const rear = kit.axles.get("rear")!;
  h.vehicle.chassisBody.setLinvel({ x: 4.5, y: 0, z: 0 }, true);
  for (const a of kit.axles.values()) {
    a.body.setLinvel({ x: 4.5, y: 0, z: 0 }, true);
  }
  let maxYaw = 0;
  let maxSpeed = 0;
  let minUpright = 1;
  for (let i = 0; i < 180; i += 1) {
    step(h, { throttle: 1, steer: 0, reset: false });
    maxYaw = Math.max(
      maxYaw,
      axleYawAbs(h.vehicle.chassisBody, front.body),
      axleYawAbs(h.vehicle.chassisBody, rear.body)
    );
    maxSpeed = Math.max(maxSpeed, speed(h.vehicle));
    minUpright = Math.min(minUpright, uprightY(h.vehicle));
  }
  check(failures, "ram_axle_yaw", maxYaw <= MAX_AXLE_YAW_RAM, `maxYaw=${maxYaw.toFixed(3)} need<=${MAX_AXLE_YAW_RAM}`);
  check(failures, "ram_upright_min", minUpright >= 0.35, `minUpright=${minUpright.toFixed(3)}`);
  check(failures, "ram_no_explode", maxSpeed <= MAX_SPEED, `maxSpeed=${maxSpeed.toFixed(3)}`);
  return {
    name: "axle_ram",
    ok: failures.length === 0,
    metrics: { maxYaw, maxSpeed, minUpright },
    failures,
  };
}

/** Flat reverse: rear axle must not fold forward under the belly. */
async function scenarioThrottleFold(): Promise<ScenarioReport> {
  const failures: GateFailure[] = [];
  const h = await createHarness();
  place(h, 0, 0.22, 3, 0);
  idle(h, 120);
  const kit = h.vehicle.kitSuspension();
  check(failures, "tfold_has_kit", !!kit, "kit missing");
  if (!kit) {
    return { name: "throttle_fold", ok: false, metrics: {}, failures };
  }
  const front = kit.axles.get("front")!;
  const rear = kit.axles.get("rear")!;
  let peakPitch = 0;
  let peakFrontZ = -999;
  let peakLinkAbs = 0;
  let minHang = 999;
  let maxSpeed = 0;
  let minUpright = 1;
  for (let i = 0; i < 240; i += 1) {
    step(h, { throttle: 1, steer: 0, reset: false });
    const chassis = h.vehicle.chassisBody;
    const lf = axleInChassis(chassis, front.body);
    const lr = axleInChassis(chassis, rear.body);
    peakFrontZ = Math.max(peakFrontZ, lf.z);
    peakPitch = Math.max(
      peakPitch,
      Math.abs(axlePitchAbs(chassis, front.body)),
      Math.abs(axlePitchAbs(chassis, rear.body))
    );
    const d = sampleSuspensionDiag(kit);
    peakLinkAbs = Math.max(peakLinkAbs, d.maxLinkAbsError);
    minHang = Math.min(minHang, chassisUpHang(chassis, front.body), chassisUpHang(chassis, rear.body));
    maxSpeed = Math.max(maxSpeed, speed(h.vehicle));
    minUpright = Math.min(minUpright, uprightY(h.vehicle));
    void lr;
  }
  check(
    failures,
    "tfold_pitch",
    peakPitch <= MAX_THROTTLE_AXLE_PITCH,
    `peakPitch=${peakPitch.toFixed(3)} need<=${MAX_THROTTLE_AXLE_PITCH}`
  );
  check(
    failures,
    "tfold_front_z",
    peakFrontZ <= MAX_THROTTLE_FRONT_LOCAL_Z,
    `peakFrontZ=${peakFrontZ.toFixed(4)} need<=${MAX_THROTTLE_FRONT_LOCAL_Z}`
  );
  check(
    failures,
    "tfold_link",
    peakLinkAbs <= MAX_THROTTLE_LINK_ABS,
    `peakLinkAbs=${peakLinkAbs.toFixed(5)} need<=${MAX_THROTTLE_LINK_ABS}`
  );
  check(failures, "tfold_hang", minHang >= 0.008, `minHang=${minHang.toFixed(3)}`);
  check(failures, "tfold_upright", minUpright >= MIN_UPRIGHT_CRAWL, `minUpright=${minUpright.toFixed(3)}`);
  check(failures, "tfold_no_explode", maxSpeed <= MAX_SPEED, `maxSpeed=${maxSpeed.toFixed(3)}`);
  return {
    name: "throttle_fold",
    ok: failures.length === 0,
    metrics: { peakPitch, peakFrontZ, peakLinkAbs, minHang, maxSpeed, minUpright },
    failures,
  };
}

function axlePitchAbs(
  chassis: { rotation: () => { x: number; y: number; z: number; w: number } },
  axle: { rotation: () => { x: number; y: number; z: number; w: number } }
): number {
  const cr = chassis.rotation();
  const ar = axle.rotation();
  const e = new THREE.Euler().setFromQuaternion(
    new THREE.Quaternion(cr.x, cr.y, cr.z, cr.w)
      .invert()
      .multiply(new THREE.Quaternion(ar.x, ar.y, ar.z, ar.w)),
    "YXZ"
  );
  return e.x;
}

async function scenarioReverseFold(): Promise<ScenarioReport> {
  const failures: GateFailure[] = [];
  const h = await createHarness();
  place(h, 0, 0.22, 3, 0);
  idle(h, 120);
  const kit = h.vehicle.kitSuspension();
  check(failures, "rev_has_kit", !!kit, "kit missing");
  if (!kit) {
    return { name: "reverse_fold", ok: false, metrics: {}, failures };
  }
  const rear = kit.axles.get("rear")!;
  const z0 = h.vehicle.chassisBody.translation().z;
  let peak = 0;
  let minLocalZR = 999;
  let maxSpeed = 0;
  let minUpright = 1;
  let minHang = 999;
  for (let i = 0; i < 280; i += 1) {
    step(h, { throttle: -1, steer: 0, reset: false });
    const chassis = h.vehicle.chassisBody;
    const t = chassis.translation();
    peak = Math.max(peak, t.z - z0);
    const lr = axleInChassis(chassis, rear.body);
    minLocalZR = Math.min(minLocalZR, lr.z);
    minHang = Math.min(minHang, chassisUpHang(chassis, rear.body));
    maxSpeed = Math.max(maxSpeed, speed(h.vehicle));
    minUpright = Math.min(minUpright, uprightY(h.vehicle));
  }
  check(failures, "rev_drive", peak >= 0.08, `peak=${peak.toFixed(3)} need>=0.08`);
  check(
    failures,
    "rev_fold_rear",
    minLocalZR >= MIN_REAR_AXLE_LOCAL_Z,
    `minLocalZR=${minLocalZR.toFixed(4)} need>=${MIN_REAR_AXLE_LOCAL_Z} (rest=+0.1565)`
  );
  check(
    failures,
    "rev_hang_rear",
    minHang >= MIN_BUMP_FRONT_HANG,
    `minHang=${minHang.toFixed(3)} need>=${MIN_BUMP_FRONT_HANG}`
  );
  check(failures, "rev_upright_min", minUpright >= MIN_UPRIGHT_CRAWL, `minUpright=${minUpright.toFixed(3)}`);
  check(failures, "rev_no_explode", maxSpeed <= MAX_SPEED, `maxSpeed=${maxSpeed.toFixed(3)}`);
  return {
    name: "reverse_fold",
    ok: failures.length === 0,
    metrics: { peak, minLocalZR, minHang, maxSpeed, minUpright },
    failures,
  };
}

async function scenarioLinkIntegrity(): Promise<ScenarioReport> {
  const failures: GateFailure[] = [];
  const h = await createHarness();
  place(h, 0, 0.22, 0, 0);
  idle(h, 90);
  const kit = h.vehicle.kitSuspension();
  check(failures, "link_has_kit", !!kit, "kit missing");
  if (!kit) {
    return { name: "link_integrity", ok: false, metrics: {}, failures };
  }
  let maxAbs = 0;
  let maxRel = 0;
  let maxLat = 0;
  const sample = (): void => {
    const d = sampleSuspensionDiag(kit);
    maxAbs = Math.max(maxAbs, d.maxLinkAbsError);
    maxRel = Math.max(maxRel, d.maxLinkRelError);
    for (const a of d.axles) maxLat = Math.max(maxLat, Math.abs(a.lateral));
  };
  sample();
  for (let i = 0; i < 180; i += 1) {
    step(h, { throttle: 1, steer: 0, reset: false });
    if (i % 10 === 0) sample();
  }
  sample();
  check(
    failures,
    "link_abs_error",
    maxAbs <= MAX_LINK_ABS_ERROR,
    `maxAbs=${maxAbs.toFixed(5)} need<=${MAX_LINK_ABS_ERROR}`
  );
  check(
    failures,
    "link_rel_error",
    maxRel <= MAX_LINK_REL_ERROR,
    `maxRel=${maxRel.toFixed(4)} need<=${MAX_LINK_REL_ERROR}`
  );
  check(
    failures,
    "link_lateral",
    maxLat <= MAX_AXLE_LATERAL,
    `maxLat=${maxLat.toFixed(4)} need<=${MAX_AXLE_LATERAL}`
  );
  return {
    name: "link_integrity",
    ok: failures.length === 0,
    metrics: { maxAbs, maxRel, maxLat },
    failures,
  };
}

const thresholds = {
  MAX_SPEED,
  MIN_UPRIGHT_IDLE,
  MIN_UPRIGHT_CRAWL,
  MIN_UPRIGHT_FINAL,
  MIN_FLAT_PEAK,
  MIN_RAMP_PEAK,
  MIN_LEDGE_PEAK,
  MIN_LEDGE_Y,
  MIN_RAMP_CLIMB_Y,
  MIN_RAMP_FROM_FLAT_CLIMB_Y,
  MIN_STAIRS_CLIMB_Y,
  MAX_LINK_ABS_ERROR,
  MAX_LINK_REL_ERROR,
  MAX_AXLE_LATERAL,
  MAX_AXLE_YAW_RAM,
  MIN_BUMP_FRONT_HANG,
  MIN_PLAY_HANG,
  MAX_AXLE_LOCAL_Y,
  MAX_AXLE_STATION_DRIFT,
  MAX_FRONT_AXLE_LOCAL_Z,
  MIN_REAR_AXLE_LOCAL_Z,
  MAX_THROTTLE_AXLE_PITCH,
  MAX_THROTTLE_FRONT_LOCAL_Z,
  MAX_THROTTLE_LINK_ABS,
};

const reports: ScenarioReport[] = [];
reports.push(await scenarioIdleUpright());
reports.push(await scenarioFlatForward());
reports.push(await scenarioRampClimb());
reports.push(await scenarioRampFromFlat());
reports.push(await scenarioStairsClimb());
reports.push(await scenarioLedgeCrest());
reports.push(await scenarioBumpHang());
reports.push(await scenarioThrottleHang());
reports.push(await scenarioThrottleFold());
reports.push(await scenarioReverseFold());
reports.push(await scenarioAxleRam());
reports.push(await scenarioLinkIntegrity());

const ok = reports.every((r) => r.ok);
console.log(JSON.stringify({ ok, thresholds, scenarios: reports }, null, 2));

if (!ok) {
  console.error("crawl-test FAILED");
  for (const r of reports) {
    if (r.ok) continue;
    for (const f of r.failures) console.error(`  [${r.name}] ${f.gate}: ${f.detail}`);
  }
  process.exit(1);
}
console.log("crawl-test OK");
