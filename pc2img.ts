// ============================================================
//  Foxglove User Script: PointCloud2 → Depth/Field Image
// ============================================================
//
//  Converts a sensor_msgs/PointCloud2 to a sensor_msgs/Image
//  using spherical (azimuth / elevation) projection.
//
//  Designed for dense, unorganised clouds with variable point
//  counts per frame (no organized ring structure assumed).
//
//  HOW TO USE
//  ----------
//  1. Paste into Foxglove Studio → User Scripts panel.
//  2. Set INPUT_TOPIC / OUTPUT_TOPIC to match your setup.
//  3. Tune the CONFIG block (see comments).
//  4. Add a Raw Image panel and subscribe to OUTPUT_TOPIC.
//
// ============================================================

import { Input, Message } from "./types";

// ─── Topics ──────────────────────────────────────────────────────────────────
export const inputs = ["/multiscan/lidar_scan"];
export const output = "/lidar_image";

// ─── Configuration ────────────────────────────────────────────────────────────
const CONFIG = {
  // ── Resolution ───────────────────────────────────────────────────────────
  // Set DYNAMIC_RESOLUTION: true  → image size is computed each frame from
  //                                  the data's angular extent and point density.
  // Set DYNAMIC_RESOLUTION: false → fixed OUTPUT_WIDTH × OUTPUT_HEIGHT pixels.
  DYNAMIC_RESOLUTION: false,
  OUTPUT_WIDTH: 512, // pixels  (ignored when DYNAMIC_RESOLUTION: true)
  OUTPUT_HEIGHT: 128, // pixels  (ignored when DYNAMIC_RESOLUTION: true)

  // ── Angular FOV (radians) ────────────────────────────────────────────────
  // Used only when DYNAMIC_RESOLUTION: false.
  //   Full 360° spinning LiDAR  →  H_FOV = 2π,  V_FOV = ~0.52 (30°)
  //   Solid-state / narrow FOV  →  set smaller values to fit your sensor
  H_FOV: 2 * Math.PI, // horizontal  (0 < H_FOV ≤ 2π)
  V_FOV: 1.57079632679, // vertical
  //
  // Azimuth zero maps to the left edge; elevation zero maps to image centre.
  // A full 360° sweep wraps so that azimuth –π and +π share the same column.

  // Rotates the azimuth before binning, in radians. Use this when the
  // sensor is mounted at an angle relative to the vehicle/robot frame, so
  // "straight ahead" lines up with the centre/edge of the image as expected.
  // Positive values rotate counter-clockwise (right-hand rule about +z).
  AZIMUTH_OFFSET: 1.57079632679,

  // ── Display field ────────────────────────────────────────────────────────
  // Which point value gets mapped to pixel colour.
  //   "depth"  – distance from the sensor origin (always available, computed
  //              from x/y/z). Uses MIN_DEPTH / MAX_DEPTH below for scaling.
  //   anything else – read directly from a PointCloud2 field with that name,
  //              e.g. "intensity", "reflectivity", "ring", "time". Uses
  //              VALUE_MIN / VALUE_MAX below for scaling. If the field
  //              doesn't exist on the cloud, falls back to "depth".
  VALUE_FIELD: "depth" as string,

  // Pixel-value scaling range when VALUE_FIELD is "depth".
  // These also define pixel value 0 (MIN_DEPTH) and 65535/white (MAX_DEPTH).
  MIN_DEPTH: 0,
  MAX_DEPTH: 6,
  DEPTH_RESCALE_POWER: 1,

  // Pixel-value scaling range when VALUE_FIELD is anything other than
  // "depth" (e.g. intensity/reflectivity units, sensor-specific).
  // Set VALUE_MIN === VALUE_MAX to auto-range per-frame from the data
  // instead of fixed bounds (handy since intensity/reflectivity scales
  // vary a lot between sensors).
  VALUE_MIN: 0,
  VALUE_MAX: 0,

  // ── Output encoding ──────────────────────────────────────────────────────
  // "mono16"  – 16-bit greyscale, little-endian; best for downstream processing.
  //             Foxglove will auto-stretch the histogram for display.
  // "rgb8"    – Jet false-colour for immediate visual inspection.
  ENCODING: "mono16" as "mono16" | "rgb8" | "32FC1",

  // ── Occlusion handling ───────────────────────────────────────────────────
  // When multiple points land on the same pixel, keep the point with the
  // smallest geometric depth (closest surface wins), regardless of which
  // field is being displayed. Set to false to keep the LAST one written
  // (faster, lower quality).
  KEEP_CLOSEST: true,

  // ── Vertical gap interpolation ───────────────────────────────────────────
  // Sparse-layer LiDARs (e.g. 14-32 beam units) leave empty rows once the
  // dynamic resolution pass picks an image height taller than the real
  // number of layers. When true, blank rows are filled by linearly
  // interpolating the displayed value, per column, between the nearest
  // filled row above and below. Only applied when DYNAMIC_RESOLUTION is true.
  INTERPOLATE_GAPS: true,

  // Largest blank run (in rows) that will be bridged. Set to Infinity to
  // always interpolate, or lower this to avoid bridging large real gaps
  // (e.g. missing returns at the top/bottom of a narrow FOV).
  MAX_GAP_ROWS: 4,
} as const;

/** Encoding string to bytes/pixel */
const bppMap = new Map([
  ["mono16", 2],
  ["rgb8", 3],
  ["32FC1", 4],
]);

// ─── PointCloud2 datatype constants ──────────────────────────────────────────
const DT = {
  INT8: 1,
  UINT8: 2,
  INT16: 3,
  UINT16: 4,
  INT32: 5,
  UINT32: 6,
  FLOAT32: 7,
  FLOAT64: 8,
} as const;

// Warn at most once per session if the configured VALUE_FIELD is missing.
let warnedMissingField = false;

// ─── Helpers ─────────────────────────────────────────────────────────────────

/** Read one value from a DataView at byte offset using the PointCloud2 datatype ID. */
function readScalar(
  view: DataView,
  byteOffset: number,
  datatype: number,
  le: boolean,
): number {
  switch (datatype) {
    case DT.FLOAT32:
      return view.getFloat32(byteOffset, le);
    case DT.FLOAT64:
      return view.getFloat64(byteOffset, le);
    case DT.INT8:
      return view.getInt8(byteOffset);
    case DT.UINT8:
      return view.getUint8(byteOffset);
    case DT.INT16:
      return view.getInt16(byteOffset, le);
    case DT.UINT16:
      return view.getUint16(byteOffset, le);
    case DT.INT32:
      return view.getInt32(byteOffset, le);
    case DT.UINT32:
      return view.getUint32(byteOffset, le);
    default:
      return NaN;
  }
}

/**
 * Jet colourmap.  t ∈ [0, 1]  →  [R, G, B] ∈ [0, 255]
 * Blue = low value, Red = high value.
 */
function jet(t: number): [number, number, number] {
  const clamp = (v: number) => Math.max(0, Math.min(255, Math.round(v * 255)));
  return [
    clamp(Math.min(4 * t - 1.5, -4 * t + 4.5)),
    clamp(Math.min(4 * t - 0.5, -4 * t + 3.5)),
    clamp(Math.min(4 * t + 0.5, -4 * t + 2.5)),
  ];
}

/**
 * Fills blank rows in `valuePx`, per column, by linearly interpolating
 * between the nearest filled row above and below. Blank runs longer than
 * `maxGap` rows are left untouched. Edge runs (no filled row on one side)
 * are also left untouched, since there is nothing to interpolate towards.
 */
function interpolateColumnGaps(
  valuePx: Float32Array,
  outW: number,
  outH: number,
  maxGap: number,
): void {
  for (let col = 0; col < outW; col++) {
    let prevRow = -1;
    let prevVal = NaN;

    for (let row = 0; row < outH; row++) {
      const idx = row * outW + col;
      const val = valuePx[idx];
      if (isNaN(val)) continue;

      if (prevRow >= 0) {
        const gap = row - prevRow - 1;
        if (gap > 0 && gap <= maxGap) {
          const span = row - prevRow;
          for (let r = prevRow + 1; r < row; r++) {
            const t = (r - prevRow) / span;
            valuePx[r * outW + col] = prevVal + (val - prevVal) * t;
          }
        }
      }
      prevRow = row;
      prevVal = val;
    }
  }
}

// ─── Main entry point ─────────────────────────────────────────────────────────
export default function script(
  event: Input<"/multiscan/lidar_scan">,
): Message<"sensor_msgs/Image"> | undefined {
  const cloud = event.message as Message<"sensor_msgs/PointCloud2">;

  // ── 1. Locate x / y / z (and, if requested, the display) fields ────────
  const fieldMap = new Map<string, { offset: number; datatype: number }>();
  for (const f of cloud.fields) {
    fieldMap.set(f.name, { offset: f.offset, datatype: f.datatype });
  }

  const xf = fieldMap.get("x");
  const yf = fieldMap.get("y");
  const zf = fieldMap.get("z");

  if (xf == null || yf == null || zf == null) {
    return undefined; // missing x/y/z fields
  }

  const useDepthAsValue = CONFIG.VALUE_FIELD === "depth";
  let valueField = useDepthAsValue ? null : fieldMap.get(CONFIG.VALUE_FIELD);

  if (!useDepthAsValue && valueField == null) {
    if (!warnedMissingField) {
      log(
        `[depth_image] Field "${CONFIG.VALUE_FIELD}" not found on PointCloud2` +
          ` (available: ${[...fieldMap.keys()].join(", ")}). Falling back to depth.`,
      );
      warnedMissingField = true;
    }
  }
  const valueFieldResolved = !useDepthAsValue && valueField != null;

  const { point_step, data, is_bigendian, header } = cloud;
  const le = !is_bigendian;
  const view = new DataView(data.buffer, data.byteOffset, data.byteLength);
  const numPoints = Math.floor(data.byteLength / point_step);

  if (numPoints === 0) return undefined;

  // When MIN_DEPTH === MAX_DEPTH the depth range is derived from the frame's data.
  const autoDepthRange = (CONFIG.MIN_DEPTH as number) === CONFIG.MAX_DEPTH;
  // Same convention for the generic value range.
  const autoValueRange = (CONFIG.VALUE_MIN as number) === CONFIG.VALUE_MAX;

  let depthMin: number = CONFIG.MIN_DEPTH;
  let depthMax: number = CONFIG.MAX_DEPTH;

  // ── 2. Pass 1 – unpack points, compute angular extents ─────────────────
  // Pre-allocate typed arrays for the maximum possible count
  const azBuf = new Float32Array(numPoints);
  const elBuf = new Float32Array(numPoints);
  const depBuf = new Float32Array(numPoints); // geometric depth, always (used for occlusion)
  const valBuf = new Float32Array(numPoints); // value actually displayed
  let valid = 0;

  let minAz = Infinity,
    maxAz = -Infinity;
  let minEl = Infinity,
    maxEl = -Infinity;
  let minDep = Infinity,
    maxDep = -Infinity;
  let minVal = Infinity,
    maxVal = -Infinity;

  for (let i = 0; i < numPoints; i++) {
    const base = i * point_step;
    const x = readScalar(view, base + xf.offset, xf.datatype, le);
    const y = readScalar(view, base + yf.offset, yf.datatype, le);
    const z = readScalar(view, base + zf.offset, zf.datatype, le);

    // Reject NaN / Inf (caller promises they're removed, but be defensive)
    if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;

    const xyDist = Math.sqrt(x * x + y * y);
    const depth = Math.sqrt(xyDist * xyDist + z * z);

    // if (autoDepthRange ? depth <= 0 : depth < depthMin || depth > depthMax) continue;

    // Spherical coordinates:
    //   azimuth   atan2(y, x)  ∈ [-π, π]   → image column
    //   elevation atan2(z, r)  ∈ [-π/2, π/2] → image row (up = low row index)
    let az = Math.atan2(y, x) + CONFIG.AZIMUTH_OFFSET;
    // Wrap back into [-π, π) so the offset can't push points off the
    // edge of the angular window.
    az =
      ((((az + Math.PI) % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI)) -
      Math.PI;
    const el = Math.atan2(z, xyDist);

    // The displayed value: depth itself, or a separate field read straight
    // from the point's bytes.
    let val: number;
    if (valueFieldResolved && valueField != null) {
      val = readScalar(view, base + valueField.offset, valueField.datatype, le);
      if (!isFinite(val)) continue;
    } else {
      val = Math.pow(depth, CONFIG.DEPTH_RESCALE_POWER);
    }

    azBuf[valid] = az;
    elBuf[valid] = el;
    depBuf[valid] = depth;
    valBuf[valid] = val;
    valid++;

    if (az < minAz) minAz = az;
    if (az > maxAz) maxAz = az;
    if (el < minEl) minEl = el;
    if (el > maxEl) maxEl = el;
    if (depth < minDep) minDep = depth;
    if (depth > maxDep) maxDep = depth;
    if (val < minVal) minVal = val;
    if (val > maxVal) maxVal = val;
  }

  if (valid === 0) return undefined;

  // Resolve auto depth range now that we've seen all valid points.
  if (autoDepthRange) {
    depthMin = minDep;
    depthMax = maxDep;
  }

  // Resolve the scaling range actually used for pixel encoding.
  let valueMin: number, valueMax: number;
  if (useDepthAsValue) {
    valueMin = depthMin;
    valueMax = depthMax;
  } else if (autoValueRange) {
    valueMin = minVal;
    valueMax = maxVal;
  } else {
    valueMin = CONFIG.VALUE_MIN;
    valueMax = CONFIG.VALUE_MAX;
  }

  // ── 3. Determine image resolution and angular window ───────────────────
  let outW: number, outH: number;
  let azOrigin: number, azRange: number;
  let elOrigin: number, elRange: number;

  if (CONFIG.DYNAMIC_RESOLUTION) {
    // Fit window exactly to this frame's data
    azOrigin = minAz;
    azRange = Math.max(maxAz - minAz, 1e-6);
    elOrigin = minEl;
    elRange = Math.max(maxEl - minEl, 1e-6);

    // Target roughly 1 point per output pixel, respect a sensible aspect ratio
    const aspect = azRange / elRange;
    outH = Math.max(16, Math.min(1024, Math.round(Math.sqrt(valid / aspect))));
    outW = Math.max(16, Math.min(4096, Math.round(outH * aspect)));
  } else {
    outW = CONFIG.OUTPUT_WIDTH;
    outH = CONFIG.OUTPUT_HEIGHT;
    azOrigin = -CONFIG.H_FOV / 2; // centre azimuth at 0°
    azRange = CONFIG.H_FOV;
    elOrigin = -CONFIG.V_FOV / 2; // centre elevation at 0°
    elRange = CONFIG.V_FOV;
  }

  // Guard against a degenerate frame where every point has the same value.
  const valueRange = Math.max(valueMax - valueMin, 1e-6);
  const bytesPerPixel = bppMap.get(CONFIG.ENCODING) as number;
  const rowStep = outW * bytesPerPixel;
  const isRaw =
    CONFIG.ENCODING.includes("32F") || CONFIG.ENCODING.includes("64F");
  const imgBuf = isRaw
    ? new Float32Array(outH * outW)
    : new Uint8Array(outH * rowStep);

  // Per-pixel value accumulator. NaN means "no point landed here yet".
  const valuePx = new Float32Array(outW * outH).fill(NaN);
  // Tracks the geometric depth of whichever point currently "owns" each
  // pixel, so occlusion is always resolved by closest-surface even when a
  // non-depth field is being displayed.
  const depthKeyPx = CONFIG.KEEP_CLOSEST
    ? new Float32Array(outW * outH).fill(Infinity)
    : null;

  // ── 4. Pass 2 – project points onto the value buffer ────────────────────
  for (let i = 0; i < valid; i++) {
    // Normalise to [0, 1] within the configured angular window
    const u = (azBuf[i] - azOrigin) / azRange;
    const v = (elBuf[i] - elOrigin) / elRange;

    if (u < 0 || u >= 1 || v < 0 || v >= 1) continue;

    // Flip horizontally: this matches the sensor's actual left/right sense.
    // (atan2(y, x) increases counter-clockwise when viewed from above, which
    // comes out mirrored compared to how the image is normally read.)
    const col = outW - 1 - Math.floor(u * outW);
    // Flip vertically: higher elevation → lower row index (top of image)
    const row = Math.floor((1.0 - v) * outH);

    if (col < 0 || col >= outW || row < 0 || row >= outH) continue;

    const pixIdx = row * outW + col;

    if (depthKeyPx != null) {
      const dep = depBuf[i];
      if (dep >= depthKeyPx[pixIdx]) continue;
      depthKeyPx[pixIdx] = dep;
    }

    valuePx[pixIdx] = valBuf[i];
  }

  // ── 5. Pass 3 – bridge blank rows left by sparse vertical layers ────────
  // Only meaningful in dynamic-resolution mode, where the chosen image
  // height is rarely an exact multiple of the sensor's real layer count.
  if (CONFIG.INTERPOLATE_GAPS) {
    interpolateColumnGaps(valuePx, outW, outH, CONFIG.MAX_GAP_ROWS);
  }

  // ── 6. Pass 4 – encode the value buffer into the output image format ───
  for (let pixIdx = 0; pixIdx < outW * outH; pixIdx++) {
    const val = valuePx[pixIdx];

    if (isRaw) {
      imgBuf[pixIdx] = isNaN(val) ? 0 : val;
      continue;
    }

    if (isNaN(val)) continue; // leave pixel as 0 (background); buffer is zero-initialised

    // Normalise to [0, 1] for encoding
    const t = Math.max(0.0, Math.min(1.0, (val - valueMin) / valueRange));

    if (CONFIG.ENCODING === "rgb8") {
      // False-colour: Jet colourmap (blue = low, red = high)
      const [r, g, b] = jet(t);
      const byteIdx = pixIdx * 3;
      imgBuf[byteIdx] = r;
      imgBuf[byteIdx + 1] = g;
      imgBuf[byteIdx + 2] = b;
    } else {
      // mono16, little-endian
      // 0x0000 = valueMin, 0xFFFF = valueMax
      const val16 = Math.round(t * 65535);
      const byteIdx = pixIdx * 2;
      imgBuf[byteIdx] = val16 & 0xff; // low byte
      imgBuf[byteIdx + 1] = (val16 >> 8) & 0xff; // high byte
    }
  }

  // ── 7. Return image message ────────────────────────────────────────────
  return {
    header,
    height: outH,
    width: outW,
    encoding: CONFIG.ENCODING,
    is_bigendian: 0,
    step: rowStep,
    data: new Uint8Array(imgBuf.buffer),
  };
}
