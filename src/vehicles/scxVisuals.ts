/**
 * Phase 5: procedural SCX10.1-ish multipart visuals (Three.js only).
 * Rapier proxies stay simple cuboids/spheres — these meshes do not collide.
 */
import * as THREE from "three";
import type { KitDef, Vec3 } from "./types.ts";

const MAT = {
  rail: new THREE.MeshStandardMaterial({ color: 0x3a3d42, metalness: 0.55, roughness: 0.45 }),
  hoop: new THREE.MeshStandardMaterial({ color: 0x2b2e33, metalness: 0.4, roughness: 0.5 }),
  skid: new THREE.MeshStandardMaterial({ color: 0x1a1c1f, metalness: 0.35, roughness: 0.55 }),
  plastic: new THREE.MeshStandardMaterial({ color: 0x111111, metalness: 0.05, roughness: 0.85 }),
  tray: new THREE.MeshStandardMaterial({ color: 0x22262c, metalness: 0.25, roughness: 0.6 }),
  bumper: new THREE.MeshStandardMaterial({ color: 0x4a4e55, metalness: 0.5, roughness: 0.4 }),
  accent: new THREE.MeshStandardMaterial({ color: 0x5c6b7a, metalness: 0.3, roughness: 0.55 }),
  axle: new THREE.MeshStandardMaterial({ color: 0x555555, metalness: 0.6, roughness: 0.4 }),
  pumpkin: new THREE.MeshStandardMaterial({ color: 0x3d3d3d, metalness: 0.45, roughness: 0.5 }),
  link: new THREE.MeshStandardMaterial({ color: 0xb08d57, metalness: 0.35, roughness: 0.55 }),
  shockBody: new THREE.MeshStandardMaterial({ color: 0x2a4a6a, metalness: 0.4, roughness: 0.45 }),
  shockShaft: new THREE.MeshStandardMaterial({ color: 0xc0c4c8, metalness: 0.8, roughness: 0.25 }),
  shockSpring: new THREE.MeshStandardMaterial({ color: 0xd4a017, metalness: 0.5, roughness: 0.4 }),
};

function box(
  sx: number,
  sy: number,
  sz: number,
  mat: THREE.Material,
  x = 0,
  y = 0,
  z = 0
): THREE.Mesh {
  const m = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), mat);
  m.position.set(x, y, z);
  return m;
}

function cyl(
  rTop: number,
  rBot: number,
  h: number,
  mat: THREE.Material,
  radial = 10
): THREE.Mesh {
  return new THREE.Mesh(new THREE.CylinderGeometry(rTop, rBot, h, radial), mat);
}

/** Kicked C-channel rail: vertical web + top/bottom flanges, tips kicked up. */
function buildCChannelRail(side: 1 | -1, length: number, railX: number): THREE.Group {
  const g = new THREE.Group();
  g.name = side > 0 ? "rail_right" : "rail_left";
  const webT = 0.0035;
  const flangeW = 0.012;
  const webH = 0.028;
  const x = side * railX;

  // Main web (longitudinal)
  g.add(box(webT, webH, length * 0.92, MAT.rail, x, 0.002, 0));

  // Top + bottom flanges
  g.add(box(flangeW, webT, length * 0.92, MAT.rail, x + side * (flangeW * 0.35), webH * 0.5, 0));
  g.add(box(flangeW, webT, length * 0.9, MAT.rail, x + side * (flangeW * 0.35), -webH * 0.45, 0));

  // Kicked tips (front / rear rise)
  const kickZ = length * 0.42;
  const kickY = 0.018;
  for (const sz of [-1, 1]) {
    const tip = box(webT * 1.1, webH * 0.55, length * 0.12, MAT.rail, x, kickY * 0.35, sz * kickZ);
    tip.rotation.x = sz * -0.22;
    g.add(tip);
    g.add(
      box(flangeW * 0.9, webT, length * 0.1, MAT.rail, x + side * (flangeW * 0.3), kickY * 0.7, sz * (kickZ + 0.01))
    );
  }

  // Cross-member stubs at mid
  g.add(box(Math.abs(x) * 0.35, webT * 1.2, webT * 2, MAT.accent, x * 0.55, -0.002, 0));
  return g;
}

/** Outboard triangular multi-hole shock hoop near a shock chassis mount. */
function buildShockHoop(mount: Vec3, side: 1 | -1): THREE.Group {
  const g = new THREE.Group();
  g.name = "shock_hoop";
  const baseX = mount.x;
  const baseY = mount.y;
  const baseZ = mount.z;
  const out = side * 0.018;

  // Vertical plate
  const plate = box(0.004, 0.055, 0.042, MAT.hoop, baseX + out * 0.3, baseY - 0.01, baseZ);
  g.add(plate);

  // Triangle brace (approximated with two angled boxes)
  const braceA = box(0.0035, 0.048, 0.01, MAT.hoop, baseX + out * 0.15, baseY - 0.005, baseZ);
  braceA.rotation.z = side * 0.55;
  g.add(braceA);
  const braceB = box(0.0035, 0.04, 0.01, MAT.hoop, baseX + out * 0.05, baseY - 0.012, baseZ + 0.008);
  braceB.rotation.z = side * -0.35;
  braceB.rotation.x = 0.25;
  g.add(braceB);

  // Multi-hole look: row of small cylinders (through-holes as dark plugs)
  for (let i = 0; i < 4; i += 1) {
    const hole = cyl(0.0028, 0.0028, 0.005, MAT.plastic, 6);
    hole.rotation.z = Math.PI / 2;
    hole.position.set(baseX + out * 0.55, baseY - 0.028 + i * 0.011, baseZ);
    g.add(hole);
  }
  // Upper shock mount ear
  g.add(box(0.014, 0.006, 0.01, MAT.accent, baseX + out * 0.2, baseY + 0.008, baseZ));
  return g;
}

function buildWebbedSkid(railX: number, length: number): THREE.Group {
  const g = new THREE.Group();
  g.name = "skid";
  const w = railX * 2 * 0.85;
  const y = -0.032;
  g.add(box(w, 0.003, length * 0.55, MAT.skid, 0, y, 0));
  // Web ribs
  for (let i = -2; i <= 2; i += 1) {
    g.add(box(0.003, 0.008, length * 0.5, MAT.skid, (i / 2) * w * 0.35, y + 0.004, 0));
  }
  for (let i = -1; i <= 1; i += 1) {
    g.add(box(w * 0.9, 0.006, 0.003, MAT.skid, 0, y + 0.004, i * length * 0.18));
  }
  return g;
}

function buildRadioBox(): THREE.Group {
  const g = new THREE.Group();
  g.name = "radio_box";
  g.add(box(0.055, 0.032, 0.048, MAT.plastic, 0, 0.028, -0.055));
  g.add(box(0.04, 0.004, 0.03, MAT.accent, 0, 0.046, -0.055)); // lid seam
  return g;
}

function buildBatteryTray(): THREE.Group {
  const g = new THREE.Group();
  g.name = "battery_tray";
  g.add(box(0.07, 0.008, 0.095, MAT.tray, 0, -0.005, 0.07));
  g.add(box(0.066, 0.022, 0.004, MAT.tray, 0, 0.008, 0.07 + 0.044)); // rear lip
  g.add(box(0.004, 0.018, 0.09, MAT.tray, -0.032, 0.006, 0.07));
  g.add(box(0.004, 0.018, 0.09, MAT.tray, 0.032, 0.006, 0.07));
  // Battery brick
  g.add(box(0.058, 0.02, 0.078, MAT.plastic, 0, 0.012, 0.068));
  return g;
}

function buildBumperPlate(zSign: 1 | -1, length: number): THREE.Group {
  const g = new THREE.Group();
  g.name = zSign < 0 ? "bumper_front" : "bumper_rear";
  const z = zSign * (length * 0.5 + 0.008);
  g.add(box(0.078, 0.022, 0.006, MAT.bumper, 0, -0.005, z));
  g.add(box(0.01, 0.028, 0.01, MAT.bumper, -0.032, 0.0, z - zSign * 0.004));
  g.add(box(0.01, 0.028, 0.01, MAT.bumper, 0.032, 0.0, z - zSign * 0.004));
  // Skid horn
  g.add(box(0.04, 0.005, 0.02, MAT.skid, 0, -0.018, z + zSign * 0.006));
  return g;
}

/**
 * Multipart chassis visual aligned to kit mounts / half-extents.
 * Parent at chassis body origin; children are chassis-local.
 */
export function buildScxChassisVisual(halfExtents: Vec3, kit: KitDef | null): THREE.Group {
  const root = new THREE.Group();
  root.name = "scx_chassis_visual";

  if (!kit) {
    root.add(
      box(halfExtents.x * 2, halfExtents.y * 2, halfExtents.z * 2, MAT.rail, 0, 0, 0)
    );
    return root;
  }

  const length = Math.max(halfExtents.z * 2, 0.28);
  const railX = 0.0375; // SCX10.1 ~75 mm rail centers

  root.add(buildCChannelRail(1, length, railX));
  root.add(buildCChannelRail(-1, length, railX));
  root.add(buildWebbedSkid(railX, length));
  root.add(buildRadioBox());
  root.add(buildBatteryTray());
  root.add(buildBumperPlate(-1, length));
  root.add(buildBumperPlate(1, length));

  // Cross braces between rails
  for (const z of [-0.06, 0.02, 0.09]) {
    root.add(box(railX * 2, 0.004, 0.01, MAT.accent, 0, 0.01, z));
  }

  const shockMounts = kit.chassis.mounts.filter((m) => m.id.includes("shock"));
  for (const m of shockMounts) {
    const side: 1 | -1 = m.offset.x >= 0 ? 1 : -1;
    root.add(buildShockHoop(m.offset, side));
  }

  return root;
}

/** Solid-axle visual: tube + center pumpkin + outer knuckles. */
export function buildScxAxleVisual(halfWidth: number): THREE.Group {
  const g = new THREE.Group();
  g.name = "scx_axle";
  const tube = cyl(0.009, 0.009, halfWidth * 2 * 0.92, MAT.axle, 12);
  tube.rotation.z = Math.PI / 2;
  g.add(tube);
  const pumpkin = box(0.038, 0.034, 0.04, MAT.pumpkin, 0, 0, 0);
  g.add(pumpkin);
  for (const sx of [-1, 1]) {
    const knuckle = box(0.022, 0.028, 0.028, MAT.axle, sx * halfWidth * 0.88, 0, 0);
    g.add(knuckle);
  }
  return g;
}

/** Link rod visual (scaled along local Z to span mounts). */
export function buildScxLinkVisual(length: number, panhard: boolean): THREE.Mesh {
  const r = panhard ? 0.0045 : 0.0055;
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(r * 2, r * 2, length), MAT.link);
  mesh.name = panhard ? "panhard" : "link_arm";
  return mesh;
}

export type ShockVisual = {
  root: THREE.Group;
  body: THREE.Mesh;
  shaft: THREE.Mesh;
  spring: THREE.Mesh;
};

/** Coilover stack; caller orients root from chassis mount → axle mount each frame. */
export function buildScxShockVisual(): ShockVisual {
  const root = new THREE.Group();
  root.name = "shock";
  const body = cyl(0.007, 0.007, 0.04, MAT.shockBody, 8);
  body.position.y = 0.02;
  const shaft = cyl(0.0035, 0.0035, 0.05, MAT.shockShaft, 6);
  shaft.position.y = -0.01;
  const spring = cyl(0.009, 0.009, 0.035, MAT.shockSpring, 8);
  spring.position.y = 0.005;
  root.add(body, shaft, spring);
  return { root, body, shaft, spring };
}

/** Orient a shock visual so +Y points from `from` to `to`, centered on the segment. */
export function syncShockVisual(
  visual: ShockVisual,
  from: THREE.Vector3,
  to: THREE.Vector3
): void {
  const mid = from.clone().add(to).multiplyScalar(0.5);
  visual.root.position.copy(mid);
  const dir = to.clone().sub(from);
  const len = Math.max(dir.length(), 0.04);
  dir.normalize();
  visual.root.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
  const bodyLen = Math.min(0.045, len * 0.45);
  const shaftLen = Math.max(0.02, len * 0.55);
  visual.body.scale.set(1, bodyLen / 0.04, 1);
  visual.body.position.y = len * 0.18;
  visual.shaft.scale.set(1, shaftLen / 0.05, 1);
  visual.shaft.position.y = -len * 0.12;
  visual.spring.scale.set(1, Math.min(len * 0.4, 0.04) / 0.035, 1);
  visual.spring.position.y = len * 0.02;
}
