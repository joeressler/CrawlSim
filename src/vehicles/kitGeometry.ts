/**
 * Development-time validation of kit link mount geometry before joint construction.
 * Observational / throw-on-error only — does not modify physics state.
 */
import type { AxleDef, KitDef, LinkDef, MountDef, MountRef, Vec3 } from "./types.ts";

const COINCIDE_EPS = 1e-5;
const SYMMETRY_EPS = 1e-4;
const MIN_LINK_LENGTH = 0.02;

export type ResolvedMount = {
  bodyKey: string;
  local: Vec3;
  /** Chassis-frame position at rest (axle offset + local when on axle). */
  chassisLocal: Vec3;
};

function add(a: Vec3, b: Vec3): Vec3 {
  return { x: a.x + b.x, y: a.y + b.y, z: a.z + b.z };
}

function sub(a: Vec3, b: Vec3): Vec3 {
  return { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z };
}

function len(a: Vec3): number {
  return Math.hypot(a.x, a.y, a.z);
}

function mountOffset(mounts: MountDef[], id: string, context: string): Vec3 {
  const found = mounts.find((m) => m.id === id);
  if (!found) throw new Error(`${context}: missing mount "${id}"`);
  return found.offset;
}

export function resolveMountRef(
  ref: MountRef,
  kit: KitDef,
  axleDefs: Map<string, AxleDef>,
  context: string
): ResolvedMount {
  if (ref.part === "chassis") {
    const local = mountOffset(kit.chassis.mounts, ref.mount, context);
    return { bodyKey: "chassis", local, chassisLocal: local };
  }
  const axle = axleDefs.get(ref.part);
  if (!axle) throw new Error(`${context}: unknown part "${ref.part}"`);
  const local = mountOffset(axle.mounts, ref.mount, context);
  return { bodyKey: axle.id, local, chassisLocal: add(axle.offset, local) };
}

/** Rest length and resolved endpoints for one link (chassis-frame geometry). */
export function linkRestGeometry(
  linkDef: LinkDef,
  kit: KitDef,
  axleDefs: Map<string, AxleDef>
): { from: ResolvedMount; to: ResolvedMount; length: number } {
  const from = resolveMountRef(linkDef.from, kit, axleDefs, `link ${linkDef.id} from`);
  const to = resolveMountRef(linkDef.to, kit, axleDefs, `link ${linkDef.id} to`);
  const length = len(sub(to.chassisLocal, from.chassisLocal));
  return { from, to, length };
}

/**
 * Validates four-link + panhard mount graph for joint construction.
 * Throws on incompatible or ambiguous geometry.
 */
export function validateKitLinkGeometry(kit: KitDef): void {
  const axleDefs = new Map(kit.axles.map((a) => [a.id, a]));
  if (kit.axles.length < 2) {
    throw new Error("kit geometry: need front and rear axles");
  }

  const pairKeys = new Set<string>();
  const byAxle = new Map<string, LinkDef[]>();

  for (const linkDef of kit.links) {
    const { from, to, length } = linkRestGeometry(linkDef, kit, axleDefs);

    if (from.bodyKey === to.bodyKey) {
      throw new Error(`link ${linkDef.id}: both ends on "${from.bodyKey}" (self-joint)`);
    }
    if (!(length >= MIN_LINK_LENGTH)) {
      throw new Error(`link ${linkDef.id}: rest length ${length} below ${MIN_LINK_LENGTH}`);
    }

    // World coincidence at identity spawn: chassisLocal endpoints are the intended rest pose.
    // Joint anchors must use body-local frames derived from these; they coincide by construction
    // when bodies start at rest offsets.
    const pair = [from.bodyKey, to.bodyKey].sort().join("|");
    const kind = linkDef.kind ?? "arm";
    const pairKey = `${pair}:${kind}:${linkDef.id}`;
    if (pairKeys.has(pairKey)) {
      throw new Error(`link ${linkDef.id}: duplicate constraint key ${pairKey}`);
    }
    pairKeys.add(pairKey);

    const axleKey = from.bodyKey === "chassis" ? to.bodyKey : from.bodyKey;
    const list = byAxle.get(axleKey) ?? [];
    list.push(linkDef);
    byAxle.set(axleKey, list);
  }

  for (const axle of kit.axles) {
    const links = byAxle.get(axle.id) ?? [];
    const arms = links.filter((l) => (l.kind ?? "arm") === "arm");
    const panhards = links.filter((l) => l.kind === "panhard");
    if (arms.length !== 4) {
      throw new Error(`kit geometry axle "${axle.id}": expected 4 arms, got ${arms.length}`);
    }
    if (panhards.length !== 1) {
      throw new Error(`kit geometry axle "${axle.id}": expected 1 panhard, got ${panhards.length}`);
    }

    const geos = arms.map((l) => linkRestGeometry(l, kit, axleDefs));
    // Upper vs lower: distinct Y on axle-side mounts.
    const axleYs = geos.map((g) => {
      const axleEnd = g.from.bodyKey === axle.id ? g.from : g.to;
      return axleEnd.local.y;
    });
    const yMin = Math.min(...axleYs);
    const yMax = Math.max(...axleYs);
    if (!(yMax - yMin > SYMMETRY_EPS)) {
      throw new Error(`kit geometry axle "${axle.id}": upper/lower mounts not distinct in Y`);
    }

    // L/R symmetry of arm lengths.
    const lengths = geos.map((g) => g.length);
    const mean = lengths.reduce((s, v) => s + v, 0) / lengths.length;
    for (const L of lengths) {
      if (Math.abs(L - mean) > SYMMETRY_EPS) {
        throw new Error(
          `kit geometry axle "${axle.id}": arm length asymmetry ${L} vs mean ${mean}`
        );
      }
    }

    const pan = linkRestGeometry(panhards[0]!, kit, axleDefs);
    const chassisEnd = pan.from.bodyKey === "chassis" ? pan.from : pan.to;
    const axleEnd = pan.from.bodyKey === "chassis" ? pan.to : pan.from;
    if (chassisEnd.bodyKey !== "chassis" || axleEnd.bodyKey !== axle.id) {
      throw new Error(`kit geometry: panhard for "${axle.id}" has invalid pivots`);
    }
    // Panhard should have meaningful lateral span.
    if (Math.abs(axleEnd.chassisLocal.x - chassisEnd.chassisLocal.x) < 0.02) {
      throw new Error(`kit geometry axle "${axle.id}": panhard lateral span too small`);
    }
  }

  // Unused but documents coincidence expectation for callers.
  void COINCIDE_EPS;
}
