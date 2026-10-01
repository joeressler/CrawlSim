/** Physics integration rate. Coil, tire, and link impulses all use this dt. */
export const FIXED_DT = 1 / 120;

/** Hitch clamp. 1/30 s is exactly four fixed steps, so a stall cannot spiral. */
const MAX_DT = 1 / 30;

export const MAX_SUBSTEPS = 4;

export function clampDt(seconds: number): number {
  if (!(seconds > 0)) return 0;
  return Math.min(seconds, MAX_DT);
}

/**
 * Run `tick` at FIXED_DT. A 1/60 s caller frame becomes two steps so existing
 * harness frame counts keep the same simulated time.
 */
export function forEachSubstep(dt: number, tick: (stepDt: number) => void): void {
  if (!(dt > 0)) return;
  let left = Math.min(dt, MAX_SUBSTEPS * FIXED_DT);
  let n = 0;
  while (left >= FIXED_DT * 0.5 && n < MAX_SUBSTEPS) {
    tick(FIXED_DT);
    left -= FIXED_DT;
    n += 1;
  }
}
