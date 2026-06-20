/** Helper for computing average bus voltage, total instantaneous current draw,
 * and total instantaneous power draw. For context refer to:
 * https://github.com/Cardinal-Space-Mining/lance-2026 */

import { Time } from "./types.ts";

export const inputs = [
  "/lance/track_left/info",
  "/lance/track_right/info",
  "/lance/trencher/info",
  "/lance/hopper_belt/info",
  "/lance/hopper_act/info",
];
export const output = "/foxglove/power_metrics";

type Output = {
  bus_voltage: number;
  total_current: number;
  est_power: number;
  sum_energy: number;
};

const currents: Record<string, number | undefined> = {};
const voltages: Record<string, number | undefined> = {};

let prevTime: Time = { sec: 0, nsec: 0 };
let prevPower: number = 0;
let totalEnergy: number = 0;

export default function script(event: any): Output | undefined {
  const topic = (event && event.topic) as string | undefined;
  const msg = event && event.message;
  if (!topic || !msg) return;

  // extract numeric field
  const current = msg.supply_current;
  const voltage = msg.bus_voltage;
  if (typeof current !== "number" || typeof voltage !== "number") return;

  currents[topic] = Math.abs(current);
  voltages[topic] = Math.abs(voltage);

  // values may be number | undefined
  const current_vals: (number | undefined)[] = inputs.map((t) => currents[t]);
  const voltage_vals: (number | undefined)[] = inputs.map((t) => voltages[t]);

  let sum_current = 0;
  current_vals.forEach((v) => {
    if (v !== undefined) {
      sum_current += v;
    }
  });

  let sum_voltage = 0;
  let num_voltage = 0;
  voltage_vals.forEach((v) => {
    if (v !== undefined) {
      sum_voltage += v;
      num_voltage += 1;
    }
  });
  const avg_voltage = num_voltage == 0 ? 0 : sum_voltage / num_voltage;

  const est_power = avg_voltage * sum_current;
  const rcvTime = event.receiveTime;
  if (prevTime.sec > 0 || prevTime.nsec > 0) {
    const dt =
      rcvTime.sec - prevTime.sec + (rcvTime.nsec - prevTime.nsec) * 1e-9;
    totalEnergy += (prevPower + est_power) * 0.5 * (dt / 3600);
  }
  prevTime = rcvTime;
  prevPower = est_power;

  return {
    bus_voltage: avg_voltage,
    total_current: sum_current,
    est_power,
    sum_energy: totalEnergy,
  };
}
