/** Extracts the quick auto control opt bit from the watchdog topic and 
 * republishes as a bool. For context refer to:
 * https://github.com/Cardinal-Space-Mining/lance-2026 */

import { Message } from "./types";

export const inputs = ["/lance/watchdog_status"];
export const output = "/foxglove/control_opts/is_quick_auto";

type OutputMsg = Message<"std_msgs/Bool">;

export default function script(event: any): OutputMsg | undefined {
  const msg = event && event.message;
  if (!msg) return;

  // const watchdog = Math.floor(msg.data / 1000);
  const opts = Math.abs(msg.data) % 1000;

  return opts & 2
    ? ({ data: true } as OutputMsg)
    : ({ data: false } as OutputMsg);
}
