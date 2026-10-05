import * as THREE from "three";
import { loadStockRig } from "../data/loadRig.ts";
import {
  applyGarageConfigToRig,
  loadGarageConfig,
  saveGarageConfig,
  type GarageConfig,
} from "../garage/catalog.ts";
import { Input } from "../input/Input.ts";
import { PhysicsWorld } from "../physics/PhysicsWorld.ts";
import { CameraRig } from "../render/CameraRig.ts";
import { GameRenderer } from "../render/Renderer.ts";
import { GarageConfigurator } from "../ui/GarageConfigurator.ts";
import { Hud } from "../ui/Hud.ts";
import { SceneMenu } from "../ui/SceneMenu.ts";
import { CrawlerVehicle } from "../vehicles/CrawlerVehicle.ts";
import type { RigDef } from "../vehicles/types.ts";
import { createWorldScene, parseSceneId } from "../world/sceneFactory.ts";
import type { SceneId, WorldScene } from "../world/WorldScene.ts";
import { clampDt, FIXED_DT, MAX_SUBSTEPS } from "./Time.ts";

const GARAGE_CAMERA_OFFSET = {
  x: 0,
  y: 0.58,
  z: 0.88,
};
const GARAGE_CAMERA_FOV = 44;
const TRAIL_CAMERA_FOV = 60;
const GARAGE_DISPLAY_YAW = Math.PI / 2;

export class Game {
  private readonly baseRig: RigDef;
  private activeRig: RigDef;
  private garageConfig: GarageConfig;
  private readonly physics: PhysicsWorld;
  private sceneId: SceneId;
  private activeScene: WorldScene;
  private vehicle: CrawlerVehicle;
  private readonly input: Input;
  private readonly cameraRig: CameraRig;
  private readonly renderer: GameRenderer;
  private readonly sceneMenu: SceneMenu;
  private readonly garageUi: GarageConfigurator;
  private readonly clock = new THREE.Clock();
  private paused = false;
  private accumulator = 0;

  constructor(host: HTMLElement) {
    this.baseRig = loadStockRig();
    this.garageConfig = loadGarageConfig();
    this.activeRig = applyGarageConfigToRig(this.baseRig, this.garageConfig);
    this.physics = PhysicsWorld.create();
    this.sceneId = parseSceneId(this.baseRig.initialSceneId);
    this.activeScene = createWorldScene(this.sceneId, this.baseRig.spawn);
    this.activeScene.activate(this.sceneContext());
    this.vehicle = this.createVehicleForScene(this.activeScene);
    this.applyGarageDisplayPose();
    this.vehicle.setLockedInPlace(this.sceneId === "garage");
    this.vehicle.applyGarageColors(this.garageConfig.linkColor, this.garageConfig.shockColor, this.garageConfig.servoColor);
    this.input = new Input();
    this.cameraRig = new CameraRig(this.baseRig.cameraOffset);
    this.applyCameraPreset(this.sceneId);
    this.renderer = new GameRenderer(host);
    new Hud(host);
    this.sceneMenu = new SceneMenu(host, this.sceneId, (nextSceneId) => {
      this.switchToScene(nextSceneId);
    });
    this.garageUi = new GarageConfigurator(host, this.garageConfig, {
      onChange: (nextConfig) => {
        this.garageConfig = nextConfig;
        saveGarageConfig(this.garageConfig);
        this.activeRig = applyGarageConfigToRig(this.baseRig, this.garageConfig);
        this.rebuildVehicleInActiveScene();
      },
    });
    this.garageUi.setVisible(this.sceneId === "garage");

    this.cameraRig.follow(this.vehicle.chassisPosition());

    addEventListener("resize", () => {
      this.cameraRig.onResize();
      this.renderer.resize();
    });

  }

  switchToScene(sceneId: SceneId): void {
    if (sceneId === this.sceneId) return;
    const ctx = this.sceneContext();
    this.vehicle.dispose(this.physics.world, this.activeScene.scene);
    this.activeScene.deactivate(ctx);
    this.activeScene.dispose(ctx);
    this.sceneId = sceneId;
    this.activeScene = createWorldScene(sceneId, this.baseRig.spawn);
    this.activeScene.activate(ctx);
    this.vehicle = this.createVehicleForScene(this.activeScene);
    this.applyGarageDisplayPose();
    this.vehicle.setLockedInPlace(this.sceneId === "garage");
    this.vehicle.applyGarageColors(this.garageConfig.linkColor, this.garageConfig.shockColor, this.garageConfig.servoColor);
    this.applyCameraPreset(this.sceneId);
    this.sceneMenu.setScene(this.sceneId);
    this.garageUi.setVisible(this.sceneId === "garage");
    this.vehicle.syncMeshes();
    this.cameraRig.follow(this.vehicle.chassisPosition());
    this.accumulator = 0;
  }

  start(): void {
    const loop = (): void => {
      const frameDt = clampDt(this.clock.getDelta());
      if (!this.paused) {
        const driveInput = this.input.poll();
        const activeInput = this.sceneId === "garage"
          ? { throttle: 0, steer: 0, reset: driveInput.reset }
          : driveInput;
        if (driveInput.reset) {
          this.reset();
          this.accumulator = 0;
        }
        this.accumulator += frameDt;
        let steps = 0;
        while (this.accumulator >= FIXED_DT * 0.5 && steps < MAX_SUBSTEPS) {
          if (!activeInput.reset) {
            this.vehicle.preStep(this.physics.world, activeInput, FIXED_DT);
          }
          this.physics.step(FIXED_DT);
          this.accumulator -= FIXED_DT;
          steps += 1;
        }
        if (steps === MAX_SUBSTEPS) this.accumulator = 0;
        this.vehicle.syncMeshes();
        this.cameraRig.follow(this.vehicle.chassisPosition());
      }
      this.renderer.render(this.activeScene.scene, this.cameraRig.camera);
      requestAnimationFrame(loop);
    };
    requestAnimationFrame(loop);
  }

  reset(): void {
    this.vehicle.reset();
    this.applyGarageDisplayPose();
  }

  private sceneContext() {
    return {
      physics: this.physics,
      defaultSpawn: this.baseRig.spawn,
    };
  }

  private createVehicleForScene(scene: WorldScene): CrawlerVehicle {
    const rigForScene: RigDef = {
      ...this.activeRig,
      spawn: { ...scene.spawn },
    };
    return new CrawlerVehicle(this.physics, scene.scene, rigForScene);
  }

  private applyCameraPreset(sceneId: SceneId): void {
    if (sceneId === "garage") {
      this.cameraRig.setOffset(GARAGE_CAMERA_OFFSET);
      this.cameraRig.setFov(GARAGE_CAMERA_FOV);
      return;
    }
    this.cameraRig.setOffset(this.baseRig.cameraOffset);
    this.cameraRig.setFov(TRAIL_CAMERA_FOV);
  }

  private applyGarageDisplayPose(): void {
    if (this.sceneId !== "garage") return;
    const spawn = this.activeScene.spawn;
    const q = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, GARAGE_DISPLAY_YAW, 0, "YXZ"));
    this.vehicle.chassisBody.setTranslation({ x: spawn.x, y: spawn.y, z: spawn.z }, true);
    this.vehicle.chassisBody.setRotation({ x: q.x, y: q.y, z: q.z, w: q.w }, true);
    this.vehicle.chassisBody.setLinvel({ x: 0, y: 0, z: 0 }, true);
    this.vehicle.chassisBody.setAngvel({ x: 0, y: 0, z: 0 }, true);
    this.vehicle.kitSuspension()?.reset(this.vehicle.chassisBody);
    this.vehicle.syncMeshes();
  }

  private rebuildVehicleInActiveScene(): void {
    const scene = this.activeScene.scene;
    this.vehicle.dispose(this.physics.world, scene);
    this.vehicle = this.createVehicleForScene(this.activeScene);
    this.applyGarageDisplayPose();
    this.vehicle.setLockedInPlace(this.sceneId === "garage");
    this.vehicle.applyGarageColors(this.garageConfig.linkColor, this.garageConfig.shockColor, this.garageConfig.servoColor);
    this.vehicle.syncMeshes();
    this.cameraRig.follow(this.vehicle.chassisPosition());
    this.accumulator = 0;
  }
}
