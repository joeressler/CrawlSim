import type { Vec3 } from "../vehicles/types.ts";
import { GarageScene } from "./GarageScene.ts";
import { TrailScene } from "./TrailScene.ts";
import { TrailTechnicalScene } from "./TrailTechnicalScene.ts";
import type { SceneId, WorldScene } from "./WorldScene.ts";

export function createWorldScene(sceneId: SceneId, spawn: Vec3): WorldScene {
  switch (sceneId) {
    case "trail-classic":
      return new TrailScene(spawn);
    case "trail-technical":
      return new TrailTechnicalScene(spawn);
    case "garage":
      return new GarageScene(spawn);
    default: {
      const _exhaustive: never = sceneId;
      throw new Error(`unknown scene id: ${String(_exhaustive)}`);
    }
  }
}

export function parseSceneId(value: string | undefined): SceneId {
  if (value === "trail-classic" || value === "trail-technical" || value === "garage") {
    return value;
  }
  return "trail-classic";
}
