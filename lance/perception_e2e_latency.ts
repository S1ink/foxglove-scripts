/** Stores the latest latency of each perception pipeline stage and publishes
 * the sum latency. For context refer to:
 * https://github.com/Cardinal-Space-Mining/Cardinal-Perception
 * https://github.com/Cardinal-Space-Mining/lance-2026 */

import { Message } from "./types";

export const inputs = [
  "/profiling/tasks/odometry/dt",
  "/profiling/tasks/mapping/dt",
  "/profiling/tasks/traversibility/dt",
  "/profiling/tasks/path_planning/dt",
];
export const output = "/foxglove/perception_latency";

type FloatMsg = Message<"std_msgs/Float64">;

const latest: Record<string, number | undefined> = {};

export default function script(event: any): FloatMsg | undefined {
  const topic = (event && event.topic) as string | undefined;
  const msg = event && event.message;
  if (!topic || !msg) return;

  // extract numeric field
  const raw = msg.data;
  if (typeof raw !== "number") return;

  latest[topic] = Math.abs(raw);

  // values may be number | undefined
  const values: (number | undefined)[] = inputs.map((t) => latest[t]);

  // wait until we have all topics
  if (values.some((v) => v === undefined)) return;

  // TS doesn't automatically narrow the element type of `values` after the `.some` check,
  // so assert it now to `number[]` because the previous guard guarantees no `undefined`.
  const definedValues = values as number[];

  // Now `definedValues` is number[], so reduce has no complaints.
  const sum = definedValues.reduce((a, b) => a + b, 0);

  // Alternative (works without the cast):
  // const sum = values.reduce<number>((acc, v) => acc + (v ?? 0), 0);

  return { data: sum } as FloatMsg;
}
