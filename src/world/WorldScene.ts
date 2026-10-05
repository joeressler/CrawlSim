import type * as THREE from "three";
import type { PhysicsWorld } from "../physics/PhysicsWorld.ts";
import type { Vec3 } from "../vehicles/types.ts";

export const SCENE_IDS = ["trail-classic", "trail-technical", "garage"] as const;

export type SceneId = (typeof SCENE_IDS)[number];

export type SceneContext = {
  physics: PhysicsWorld;
  defaultSpawn: Vec3;
};

export type WorldScene = {
  readonly id: SceneId;
  readonly scene: THREE.Scene;
  readonly spawn: Vec3;
  activate: (ctx: SceneContext) => void;
  deactivate: (ctx: SceneContext) => void;
  dispose: (ctx: SceneContext) => void;
};
