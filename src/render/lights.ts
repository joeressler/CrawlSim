import * as THREE from "three";

export function addTrailLights(scene: THREE.Scene): void {
  scene.add(new THREE.AmbientLight(0xffffff, 0.6));
  const sun = new THREE.DirectionalLight(0xffffff, 1.1);
  sun.position.set(5, 10, 2);
  scene.add(sun);
}
