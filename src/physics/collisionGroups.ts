/** Rapier InteractionGroups helpers: (membership << 16) | filter. */
export const GROUP_WORLD = 0x0001;
export const GROUP_CHASSIS = 0x0002;
/** Links: joint mass only; no self-hit and no world/chassis plant. */
export const GROUP_LINK = 0x0004;
/** Soft hub plant spheres — only vertical plant (no chassis-ray spring). */
export const GROUP_HUB = 0x0008;
/** Axle tubes: crumple stop vs chassis only (not world — hubs plant). */
export const GROUP_AXLE = 0x0010;

export function interactionGroups(membership: number, filter: number): number {
  return ((membership & 0xffff) << 16) | (filter & 0xffff);
}

/** Chassis hits world + axles (hard crumple stop). Not links/hubs. */
export const CHASSIS_GROUPS = interactionGroups(GROUP_CHASSIS, GROUP_WORLD | GROUP_AXLE);

/** Links: no collision (Strategy B sphericals own locate). */
export const KIT_PART_GROUPS = interactionGroups(GROUP_LINK, 0);

/**
 * Axle cuboids: collide with chassis only.
 * Rest geometry keeps a small gap; contact engages when rails fold onto the tube.
 */
export const AXLE_GROUPS = interactionGroups(GROUP_AXLE, GROUP_CHASSIS);

/** Hub spheres: plant on world only. */
export const HUB_GROUPS = interactionGroups(GROUP_HUB, GROUP_WORLD);

/** Ground / trail: hit chassis + hubs. */
export const WORLD_GROUPS = interactionGroups(GROUP_WORLD, GROUP_CHASSIS | GROUP_HUB | GROUP_WORLD);
