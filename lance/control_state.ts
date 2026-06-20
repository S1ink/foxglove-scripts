/** Extracts control mode encoded in watchdog signal and republishes as an
 * integer which can be easily used with foxglove widgets. For context refer to:
 * https://github.com/Cardinal-Space-Mining/lance-2026 */

import { Message } from "./types";

export const inputs = ["/lance/watchdog_status"];
export const output = "/foxglove/control_state";

type OutputMsg = Message<"std_msgs/Int32">;

export default function script(event: any): OutputMsg | undefined {
  const msg = event && event.message;
  if (!msg) return;

  const watchdog = Math.floor(msg.data / 1000);
  const opts = Math.abs(msg.data) % 1000;

  if (watchdog == 0) return { data: 0 } as OutputMsg;
  if (watchdog > 0) {
    return opts & 1 ? ({ data: 2 } as OutputMsg) : ({ data: 1 } as OutputMsg);
  } else {
    return opts & 1 ? ({ data: -2 } as OutputMsg) : ({ data: -1 } as OutputMsg);
  }
}
