import type { SceneId } from "../world/WorldScene.ts";

type SceneOption = {
  id: SceneId;
  label: string;
};

const SCENE_OPTIONS: SceneOption[] = [
  { id: "trail-classic", label: "Trail: Classic" },
  { id: "trail-technical", label: "Trail: Technical" },
  { id: "garage", label: "Garage" },
];

export class SceneMenu {
  private readonly root: HTMLDivElement;
  private readonly select: HTMLSelectElement;

  constructor(host: HTMLElement, initialSceneId: SceneId, onSelect: (sceneId: SceneId) => void) {
    this.root = document.createElement("div");
    this.root.className = "scene-menu";

    const label = document.createElement("label");
    label.className = "scene-menu__label";
    label.textContent = "Scene";

    this.select = document.createElement("select");
    this.select.className = "scene-menu__select";

    for (const option of SCENE_OPTIONS) {
      const el = document.createElement("option");
      el.value = option.id;
      el.textContent = option.label;
      this.select.appendChild(el);
    }

    this.setScene(initialSceneId);
    this.select.addEventListener("change", () => {
      const selected = this.select.value as SceneId;
      onSelect(selected);
    });

    label.appendChild(this.select);
    this.root.appendChild(label);
    host.appendChild(this.root);
  }

  setScene(sceneId: SceneId): void {
    this.select.value = sceneId;
  }
}
