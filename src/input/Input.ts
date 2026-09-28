export type DriveInput = {
  throttle: number;
  steer: number;
  reset: boolean;
};

const GAME_CODES = new Set([
  "KeyW",
  "KeyA",
  "KeyS",
  "KeyD",
  "KeyR",
  "ArrowUp",
  "ArrowDown",
  "ArrowLeft",
  "ArrowRight",
]);

export class Input {
  private readonly keys = new Set<string>();

  constructor() {
    const onDown = (e: KeyboardEvent): void => {
      this.keys.add(e.code);
      if (GAME_CODES.has(e.code)) e.preventDefault();
    };
    const onUp = (e: KeyboardEvent): void => {
      this.keys.delete(e.code);
    };
    window.addEventListener("keydown", onDown, true);
    window.addEventListener("keyup", onUp, true);
  }

  poll(): DriveInput {
    let throttle = 0;
    let steer = 0;
    if (this.keys.has("KeyW") || this.keys.has("ArrowUp")) throttle += 1;
    if (this.keys.has("KeyS") || this.keys.has("ArrowDown")) throttle -= 1;
    if (this.keys.has("KeyA") || this.keys.has("ArrowLeft")) steer += 1;
    if (this.keys.has("KeyD") || this.keys.has("ArrowRight")) steer -= 1;
    return {
      throttle,
      steer,
      reset: this.keys.has("KeyR"),
    };
  }
}
