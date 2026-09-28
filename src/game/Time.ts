const MAX_DT = 1 / 30;

export function clampDt(seconds: number): number {
  return Math.min(seconds, MAX_DT);
}
