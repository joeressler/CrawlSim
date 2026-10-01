import * as THREE from "three";
import { loadStockRig } from "../data/loadRig.ts";
import { Input } from "../input/Input.ts";
import { PhysicsWorld } from "../physics/PhysicsWorld.ts";
import { CameraRig } from "../render/CameraRig.ts";
import { GameRenderer } from "../render/Renderer.ts";
import { Hud } from "../ui/Hud.ts";
import { CrawlerVehicle } from "../vehicles/CrawlerVehicle.ts";
import { TrailScene } from "../world/TrailScene.ts";
import { clampDt, FIXED_DT, MAX_SUBSTEPS } from "./Time.ts";

export class Game {
  private readonly physics: PhysicsWorld;
  private readonly trail: TrailScene;
  private readonly vehicle: CrawlerVehicle;
  private readonly input: Input;
  private readonly cameraRig: CameraRig;
  private readonly renderer: GameRenderer;
  private readonly clock = new THREE.Clock();
  private paused = false;
  private accumulator = 0;

  constructor(host: HTMLElement) {
    const rig = loadStockRig();
    this.physics = PhysicsWorld.create();
    this.trail = new TrailScene(this.physics, rig.spawn);
    this.vehicle = new CrawlerVehicle(this.physics, this.trail.scene, rig);
    this.input = new Input();
    this.cameraRig = new CameraRig(rig.cameraOffset);
    this.renderer = new GameRenderer(host);
    new Hud(host);

    addEventListener("resize", () => {
      this.cameraRig.onResize();
      this.renderer.resize();
    });
  }

  start(): void {
    const loop = (): void => {
      const frameDt = clampDt(this.clock.getDelta());
      if (!this.paused) {
        const driveInput = this.input.poll();
        if (driveInput.reset) {
          this.reset();
          this.accumulator = 0;
        }
        this.accumulator += frameDt;
        let steps = 0;
        while (this.accumulator >= FIXED_DT * 0.5 && steps < MAX_SUBSTEPS) {
          if (!driveInput.reset) {
            this.vehicle.preStep(this.physics.world, driveInput, FIXED_DT);
          }
          this.physics.step(FIXED_DT);
          this.accumulator -= FIXED_DT;
          steps += 1;
        }
        if (steps === MAX_SUBSTEPS) this.accumulator = 0;
        this.vehicle.syncMeshes();
        this.cameraRig.follow(this.vehicle.chassisPosition());
      }
      this.renderer.render(this.trail.scene, this.cameraRig.camera);
      requestAnimationFrame(loop);
    };
    requestAnimationFrame(loop);
  }

  reset(): void {
    this.vehicle.reset();
  }
}
