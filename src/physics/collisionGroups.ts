/** Rapier InteractionGroups helpers: (membership << 16) | filter. */
export const GROUP_WORLD = 0x0001;
export const GROUP_CHASSIS = 0x0002;
export const GROUP_KIT = 0x0004;
/** Soft hub plant spheres — only vertical plant (no chassis-ray spring). */
export const GROUP_HUB = 0x0008;

export function interactionGroups(membership: number, filter: number): number {
  return ((membership & 0xffff) << 16) | (filter & 0xffff);
}

/** Chassis hits world only — not axles/links/hubs. */
export const CHASSIS_GROUPS = interactionGroups(GROUP_CHASSIS, GROUP_WORLD);

/** Axles/links: joint mass only; no self-hit and no world plant. */
export const KIT_PART_GROUPS = interactionGroups(GROUP_KIT, 0);

/** Hub spheres: plant on world only. */
export const HUB_GROUPS = interactionGroups(GROUP_HUB, GROUP_WORLD);

/** Ground / trail: hit chassis + hubs. */
export const WORLD_GROUPS = interactionGroups(GROUP_WORLD, GROUP_CHASSIS | GROUP_HUB | GROUP_WORLD);