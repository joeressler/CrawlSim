/**
 * Headless autonomous crawler suite (no browser).
 *
 *   npm run crawl-test
 *
 * Gates catch explode/orbit (|v|), tip-over (upright floor), and scripted
 * progress on flat / toward ramp / at ledge. Peak progress is used because
 * Phase 6: mild drive-only upright restore; crawl upright floors 0.40.
 */
import { runIdleSettle } from "./settleKit.ts";
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

const MAX_SPEED = 8;
const MIN_UPRIGHT_IDLE = 0.75;
/** Not inverted / rolling cage — long throttle currently settles ~0.35 pitch. */
const MIN_UPRIGHT_CRAWL = 0.40;
const MIN_UPRIGHT_FINAL = 0.40;

const MIN_FLAT_PEAK = 0.08;
const MIN_RAMP_PEAK = 0.08;
const MIN_LEDGE_PEAK = 0.25;
const MIN_LEDGE_Y = 0.12;

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

/** Drive from spawn toward TrailScene ramp (-Z); peak forward progress + safety. */
async function scenarioRampClimb(): Promise<ScenarioReport> {
  const failures: GateFailure[] = [];
  const h = await createHarness();
  idle(h, 300);
  const pre = uprightY(h.vehicle);
  check(failures, "ramp_pre_upright", pre >= 0.7, `pre=${pre.toFixed(3)}`);
  const z0 = h.vehicle.chassisBody.translation().z;
  let peak = 0;
  let maxY = h.vehicle.chassisBody.translation().y;
  let maxSpeed = 0;
  let minUpright = 1;
  for (let i = 0; i < 200; i += 1) {
    step(h, { throttle: 1, steer: 0, reset: false });
    const t = h.vehicle.chassisBody.translation();
    peak = Math.max(peak, z0 - t.z);
    maxY = Math.max(maxY, t.y);
    maxSpeed = Math.max(maxSpeed, speed(h.vehicle));
    minUpright = Math.min(minUpright, uprightY(h.vehicle));
  }
  const upright = uprightY(h.vehicle);
  check(failures, "ramp_peak", peak >= MIN_RAMP_PEAK, `peak=${peak.toFixed(3)} need>=${MIN_RAMP_PEAK}`);
  check(failures, "ramp_upright_min", minUpright >= MIN_UPRIGHT_CRAWL, `minUpright=${minUpright.toFixed(3)}`);
  check(failures, "ramp_upright_final", upright >= MIN_UPRIGHT_FINAL, `upright=${upright.toFixed(3)}`);
  check(failures, "ramp_no_explode", maxSpeed <= MAX_SPEED, `maxSpeed=${maxSpeed.toFixed(3)}`);
  return {
    name: "ramp_climb",
    ok: failures.length === 0,
    metrics: { peak, maxY, maxSpeed, minUpright, upright, z0, pre },
    failures,
  };
}

/** Face +X into ledge; peak +X progress and/or Y lift = crest attempt. */
async function scenarioLedgeCrest(): Promise<ScenarioReport> {
  const failures: GateFailure[] = [];
  const h = await createHarness();
  place(h, 1.8, 0.22, 2.0, -Math.PI / 2);
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

const thresholds = {
  MAX_SPEED,
  MIN_UPRIGHT_IDLE,
  MIN_UPRIGHT_CRAWL,
  MIN_UPRIGHT_FINAL,
  MIN_FLAT_PEAK,
  MIN_RAMP_PEAK,
  MIN_LEDGE_PEAK,
  MIN_LEDGE_Y,
};

const reports: ScenarioReport[] = [];
reports.push(await scenarioIdleUpright());
reports.push(await scenarioFlatForward());
reports.push(await scenarioRampClimb());
reports.push(await scenarioLedgeCrest());

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

