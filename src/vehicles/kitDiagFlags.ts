/**
 * Temporary diagnostics masks for fold investigation.
 * Production path leaves all flags false.
 */
export type KitDiagFlags = {
  skipCoilovers: boolean;
  skipHubDrive: boolean;
  skipSoftPlanar: boolean;
};

export const kitDiagFlags: KitDiagFlags = {
  skipCoilovers: false,
  skipHubDrive: false,
  skipSoftPlanar: false,
};

export function resetKitDiagFlags(): void {
  kitDiagFlags.skipCoilovers = false;
  kitDiagFlags.skipHubDrive = false;
  kitDiagFlags.skipSoftPlanar = false;
}
