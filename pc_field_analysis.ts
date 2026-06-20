/** Helper for extracting and analyzing certain pointcloud fields. Currently
 * setup to publish the min or max point timestamp of a cloud.
 * For context refer to: 
 * https://github.com/Cardinal-Space-Mining/multiscan-driver */

import { Message } from "./types";

// Subscribe to a PointCloud2 topic
export const inputs = ["/multiscan/lidar_scan"];
// Publish min/max timestamps
export const output = "/pointcloud_timestamps";

import { readPoints } from "./pointClouds";
// import type { sensor_msgs__PointCloud2 } from "./pointClouds";

type FloatMsg = Message<"std_msgs/Float64">;

export default function script(event: any): FloatMsg | undefined {
  const msg = event && event.message;
  if (!msg?.fields || !msg?.data) {
    log("no data!");
    return;
  }

  // Map field names to their indices
  const fieldNames = msg.fields.map((f: any) => f.name);
  const idxTl = fieldNames.indexOf("tl");
  const idxTh = fieldNames.indexOf("th");
  if (idxTl === -1 || idxTh === -1) {
    log("Missing tl/th fields in PointCloud2");
    return;
  }

  const points = readPoints(msg);
  if (!points.length) {
    return;
  }

  let minTimestamp = Number.MAX_SAFE_INTEGER;
  let maxTimestamp = 0;

  for (const p of points) {
    const tl = Number(p[idxTl]);
    const th = Number(p[idxTh]);
    if (Number.isNaN(tl) || Number.isNaN(th)) continue;

    // combine high and low into 64-bit integer
    const timestamp = th * 2 ** 32 + tl;

    if (timestamp < minTimestamp) minTimestamp = timestamp;
    if (timestamp > maxTimestamp) maxTimestamp = timestamp;
  }

  // Return an object; Foxglove will publish it as a JSON message
  return { data: minTimestamp } as FloatMsg;
}
