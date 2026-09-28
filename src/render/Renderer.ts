import * as THREE from "three";

export class GameRenderer {
  readonly renderer: THREE.WebGLRenderer;

  constructor(host: HTMLElement) {
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(innerWidth, innerHeight);
    this.renderer.setPixelRatio(devicePixelRatio);
    host.appendChild(this.renderer.domElement);
    addEventListener("resize", () => {
      this.resize();
    });
  }

  resize(camera?: THREE.PerspectiveCamera): void {
    this.renderer.setSize(innerWidth, innerHeight);
    if (camera) {
      camera.aspect = innerWidth / innerHeight;
      camera.updateProjectionMatrix();
    }
  }

  render(scene: THREE.Scene, camera: THREE.Camera): void {
    this.renderer.render(scene, camera);
  }
}
