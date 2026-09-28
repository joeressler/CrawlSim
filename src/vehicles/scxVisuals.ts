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
  shockBody: new THREE.MeshStandardMaterial({ color: 0x1e4d7a, metalness: 0.55, roughness: 0.35 }),
  shockCap: new THREE.MeshStandardMaterial({ color: 0x1a1a1a, metalness: 0.4, roughness: 0.5 }),
  shockCollar: new THREE.MeshStandardMaterial({ color: 0x2a2a2a, metalness: 0.5, roughness: 0.4 }),
  shockShaft: new THREE.MeshStandardMaterial({ color: 0xd8dde2, metalness: 0.9, roughness: 0.18 }),
  shockSpring: new THREE.MeshStandardMaterial({ color: 0xc9a227, metalness: 0.65, roughness: 0.35 }),
  shockEyelet: new THREE.MeshStandardMaterial({ color: 0x222222, metalness: 0.15, roughness: 0.7 }),
  shockBall: new THREE.MeshStandardMaterial({ color: 0x3a3a3a, metalness: 0.2, roughness: 0.65 }),
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

/** Outboard triangular multi-hole shock hoop with clevis ear at the kit mount. */
function buildShockHoop(mount: Vec3, side: 1 | -1): THREE.Group {
  const g = new THREE.Group();
  g.name = "shock_hoop";
  const baseX = mount.x;
  const baseY = mount.y;
  const baseZ = mount.z;
  const out = side * 0.018;

  // Vertical multi-hole plate
  const plate = box(0.004, 0.055, 0.042, MAT.hoop, baseX + out * 0.3, baseY - 0.01, baseZ);
  g.add(plate);

  const braceA = box(0.0035, 0.048, 0.01, MAT.hoop, baseX + out * 0.15, baseY - 0.005, baseZ);
  braceA.rotation.z = side * 0.55;
  g.add(braceA);
  const braceB = box(0.0035, 0.04, 0.01, MAT.hoop, baseX + out * 0.05, baseY - 0.012, baseZ + 0.008);
  braceB.rotation.z = side * -0.35;
  braceB.rotation.x = 0.25;
  g.add(braceB);

  for (let i = 0; i < 4; i += 1) {
    const hole = cyl(0.0028, 0.0028, 0.005, MAT.plastic, 6);
    hole.rotation.z = Math.PI / 2;
    hole.position.set(baseX + out * 0.55, baseY - 0.028 + i * 0.011, baseZ);
    g.add(hole);
  }

  // Clevis fork at the kit shock mount (eyelet bolts through)
  const forkGap = 0.007;
  const fork = new THREE.Group();
  fork.name = "shock_clevis_upper";
  fork.position.set(baseX, baseY, baseZ);
  for (const sz of [-1, 1]) {
    const ear = box(0.012, 0.01, 0.0035, MAT.accent, 0, 0, sz * (forkGap * 0.5 + 0.00175));
    fork.add(ear);
  }
  // Clevis pin
  const pin = cyl(0.0016, 0.0016, forkGap + 0.008, MAT.shockShaft, 6);
  pin.rotation.x = Math.PI / 2;
  fork.add(pin);
  g.add(fork);
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

/** Solid-axle visual: tube + pumpkin + knuckles + lower shock clevis tabs. */
export function buildScxAxleVisual(halfWidth: number, shockMounts: Vec3[] = []): THREE.Group {
  const g = new THREE.Group();
  g.name = "scx_axle";
  const tube = cyl(0.009, 0.009, halfWidth * 2 * 0.92, MAT.axle, 12);
  tube.rotation.z = Math.PI / 2;
  g.add(tube);
  g.add(box(0.038, 0.034, 0.04, MAT.pumpkin, 0, 0, 0));
  for (const sx of [-1, 1]) {
    g.add(box(0.022, 0.028, 0.028, MAT.axle, sx * halfWidth * 0.88, 0, 0));
  }
  // Lower shock mount clevis tabs at kit axle shock mounts
  for (const m of shockMounts) {
    const tab = new THREE.Group();
    tab.name = "shock_clevis_lower";
    tab.position.set(m.x, m.y, m.z);
    const gap = 0.007;
    for (const sz of [-1, 1]) {
      tab.add(box(0.014, 0.01, 0.0032, MAT.accent, 0, 0, sz * (gap * 0.5 + 0.0016)));
    }
    const pin = cyl(0.0015, 0.0015, gap + 0.007, MAT.shockShaft, 6);
    pin.rotation.x = Math.PI / 2;
    tab.add(pin);
    g.add(tab);
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

/** SCX-style oil shock (~80–100 mm hole-to-hole at 1/10). */
export type ShockVisual = {
  root: THREE.Group;
  body: THREE.Mesh;
  cap: THREE.Mesh;
  collar: THREE.Mesh;
  shaft: THREE.Mesh;
  spring: THREE.Mesh;
  lowerPerch: THREE.Mesh;
  upperEye: THREE.Group;
  lowerEye: THREE.Group;
  /** Reference spring height used for compress scale. */
  springRefH: number;
  bodyRefH: number;
  shaftRefH: number;
};

const SHOCK_BODY_R = 0.006;
const SHOCK_BODY_H = 0.036;
const SHOCK_SHAFT_R = 0.00155;
const SHOCK_SHAFT_H = 0.055;
const SHOCK_SPRING_R = 0.0088;
const SHOCK_SPRING_TUBE = 0.00115;
const SHOCK_SPRING_H = 0.04;
const SHOCK_SPRING_TURNS = 8;
const SHOCK_EYE_ALLOW = 0.007;

function buildCoilSpring(radius: number, tubeR: number, height: number, turns: number, mat: THREE.Material): THREE.Mesh {
  const pts: THREE.Vector3[] = [];
  const segs = Math.max(24, Math.round(turns * 14));
  for (let i = 0; i <= segs; i += 1) {
    const t = i / segs;
    const ang = t * turns * Math.PI * 2;
    pts.push(new THREE.Vector3(Math.cos(ang) * radius, (t - 0.5) * height, Math.sin(ang) * radius));
  }
  const curve = new THREE.CatmullRomCurve3(pts);
  const geo = new THREE.TubeGeometry(curve, segs, tubeR, 5, false);
  const mesh = new THREE.Mesh(geo, mat);
  mesh.name = "shock_spring";
  return mesh;
}

function buildShockEyelet(name: string): THREE.Group {
  const g = new THREE.Group();
  g.name = name;
  // Nylon/plastic rod-end housing
  const housing = cyl(0.0042, 0.0036, 0.008, MAT.shockEyelet, 8);
  g.add(housing);
  // Spherical ball (clevis pin bearing)
  const ball = new THREE.Mesh(new THREE.SphereGeometry(0.0032, 8, 6), MAT.shockBall);
  g.add(ball);
  // Eye ring (hole axis = local Z so pin through clevis fork is transverse)
  const ring = new THREE.Mesh(new THREE.TorusGeometry(0.0038, 0.0011, 6, 12), MAT.shockEyelet);
  ring.rotation.y = Math.PI / 2;
  g.add(ring);
  return g;
}

/** Coilover stack; sync orients root from chassis mount → axle mount each frame. */
export function buildScxShockVisual(): ShockVisual {
  const root = new THREE.Group();
  root.name = "shock";

  const upperEye = buildShockEyelet("shock_eye_upper");
  const lowerEye = buildShockEyelet("shock_eye_lower");

  const cap = cyl(SHOCK_BODY_R * 1.05, SHOCK_BODY_R * 1.05, 0.005, MAT.shockCap, 10);
  cap.name = "shock_cap";

  const body = cyl(SHOCK_BODY_R, SHOCK_BODY_R, SHOCK_BODY_H, MAT.shockBody, 12);
  body.name = "shock_body";
  // Threaded look: thin rings on body
  for (let i = 0; i < 6; i += 1) {
    const ring = cyl(SHOCK_BODY_R * 1.04, SHOCK_BODY_R * 1.04, 0.0012, MAT.shockCollar, 10);
    ring.position.y = -SHOCK_BODY_H * 0.35 + i * 0.0045;
    body.add(ring);
  }

  const collar = cyl(SHOCK_BODY_R * 1.15, SHOCK_BODY_R * 1.15, 0.006, MAT.shockCollar, 10);
  collar.name = "shock_collar";

  const spring = buildCoilSpring(SHOCK_SPRING_R, SHOCK_SPRING_TUBE, SHOCK_SPRING_H, SHOCK_SPRING_TURNS, MAT.shockSpring);

  const lowerPerch = cyl(SHOCK_SPRING_R * 1.05, SHOCK_SPRING_R * 0.95, 0.004, MAT.shockCollar, 10);
  lowerPerch.name = "shock_perch";

  const shaft = cyl(SHOCK_SHAFT_R, SHOCK_SHAFT_R, SHOCK_SHAFT_H, MAT.shockShaft, 8);
  shaft.name = "shock_shaft";

  root.add(upperEye, cap, body, collar, spring, lowerPerch, shaft, lowerEye);
  return {
    root,
    body,
    cap,
    collar,
    shaft,
    spring,
    lowerPerch,
    upperEye,
    lowerEye,
    springRefH: SHOCK_SPRING_H,
    bodyRefH: SHOCK_BODY_H,
    shaftRefH: SHOCK_SHAFT_H,
  };
}

const shockDir = new THREE.Vector3();
const shockY = new THREE.Vector3(0, 1, 0);

/**
 * Place shock so upper eyelet sits on chassis mount and lower eyelet on axle mount.
 * Body stays near the hoop; shaft telescopes; spring compresses with hole-to-hole length.
 */
export function syncShockVisual(visual: ShockVisual, from: THREE.Vector3, to: THREE.Vector3): void {
  shockDir.copy(to).sub(from);
  const len = Math.max(shockDir.length(), 0.045);
  shockDir.multiplyScalar(1 / len);

  visual.root.position.copy(from);
  visual.root.quaternion.setFromUnitVectors(shockY, shockDir);

  const eye = SHOCK_EYE_ALLOW;
  const usable = Math.max(0.03, len - eye * 2);
  const bodyH = Math.min(visual.bodyRefH, usable * 0.52);
  const shaftTravel = Math.max(0.012, usable - bodyH * 0.55);
  const springH = Math.max(0.012, Math.min(visual.springRefH, usable * 0.55));

  // Upper eyelet at mount
  visual.upperEye.position.set(0, eye * 0.35, 0);

  // Cap just below upper eye
  const bodyTop = eye + 0.002;
  visual.cap.position.set(0, bodyTop + 0.0025, 0);
  visual.body.scale.set(1, bodyH / visual.bodyRefH, 1);
  visual.body.position.set(0, bodyTop + 0.0025 + bodyH * 0.5, 0);

  // Preload collar near bottom of body (spring seat on body)
  const collarY = bodyTop + bodyH * 0.88;
  visual.collar.position.set(0, collarY, 0);

  // Spring between collar and lower perch
  const perchY = len - eye - 0.004;
  const springMid = (collarY + perchY) * 0.5;
  visual.spring.scale.set(1, springH / visual.springRefH, 1);
  visual.spring.position.set(0, springMid, 0);
  visual.lowerPerch.position.set(0, perchY, 0);

  // Shaft from inside body toward lower eye
  visual.shaft.scale.set(1, shaftTravel / visual.shaftRefH, 1);
  visual.shaft.position.set(0, perchY - shaftTravel * 0.35, 0);

  // Lower eyelet at axle mount
  visual.lowerEye.position.set(0, len - eye * 0.35, 0);
}
