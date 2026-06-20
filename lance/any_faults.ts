/** Keeps a running global OR of all motor fault states and republishes as bool.
 * For context refer to:
 * https://github.com/Cardinal-Space-Mining/lance-2026 */

import { Message } from "./types";

export const inputs = [
  "/lance/track_left/faults",
  "/lance/track_right/faults",
  "/lance/trencher/faults",
  "/lance/hopper_belt/faults",
  "/lance/hopper_act/faults",
];
export const output = "/foxglove/any_faults";

type BoolMsg = Message<"std_msgs/Bool">;

const latest: Record<string, boolean | undefined> = {};

export default function script(event: any): BoolMsg | undefined {
  const topic = (event && event.topic) as string | undefined;
  const msg = event && event.message;
  if (!topic || !msg) return;

  // extract numeric field
  const raw = msg.stator_current_limit_fault;
  if (typeof raw !== "boolean") return;

  latest[topic] = raw;

  // values may be number | undefined
  const values: (boolean | undefined)[] = inputs.map((t) => latest[t]);

  // wait until we have all topics
  if (values.some((v) => v === undefined)) return;

  // TS doesn't automatically narrow the element type of `values` after the `.some` check,
  // so assert it now to `number[]` because the previous guard guarantees no `undefined`.
  const definedValues = values as boolean[];

  // Now `definedValues` is number[], so reduce has no complaints.
  const or = definedValues.reduce((a, b) => a || b, false);

  // Alternative (works without the cast):
  // const sum = values.reduce<number>((acc, v) => acc + (v ?? 0), 0);

  return { data: or } as BoolMsg;
}
