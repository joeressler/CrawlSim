/** Rapier InteractionGroups helpers: (membership << 16) | filter. */
export const GROUP_WORLD = 0x0001;
export const GROUP_CHASSIS = 0x0002;
export const GROUP_KIT = 0x0004;

export function interactionGroups(membership: number, filter: number): number {
  return ((membership & 0xffff) << 16) | (filter & 0xffff);
}

/** Chassis hits world only — not axles/links. */
export const CHASSIS_GROUPS = interactionGroups(GROUP_CHASSIS, GROUP_WORLD);

/** Axles/links: mass for joints only; no self-hit and no world plant. */
export const KIT_PART_GROUPS = interactionGroups(GROUP_KIT, 0);

/** Ground / trail colliders: hit chassis (and anything still on default that includes WORLD). */
export const WORLD_GROUPS = interactionGroups(GROUP_WORLD, GROUP_CHASSIS | GROUP_WORLD);