import RAPIER from "@dimforge/rapier3d-compat";

export class PhysicsWorld {
  readonly world: RAPIER.World;

  private constructor() {
    this.world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  }

  static async init(): Promise<void> {
    await RAPIER.init();
  }

  static create(): PhysicsWorld {
    return new PhysicsWorld();
  }

  step(dt: number): void {
    this.world.timestep = dt;
    this.world.step();
  }
}
