import stock from "./rigs/stock.json" with { type: "json" };
import type { RigDef } from "../vehicles/types.ts";

export function loadStockRig(): RigDef {
  return stock as RigDef;
}
