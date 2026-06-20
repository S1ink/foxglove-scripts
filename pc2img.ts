// ============================================================
//  Foxglove User Script: PointCloud2 → Multiple Depth/Field Images
// ============================================================
//
//  Converts a sensor_msgs/PointCloud2 to a custom message containing
//  an array of sensor_msgs/Image-compatible entries, one per entry in
//  CONFIGS below.  Each entry is rendered independently using spherical
//  (azimuth / elevation) projection.
//
//  HOW TO USE
//  ----------
//  1. Paste into Foxglove Studio → User Scripts panel.
//  2. Adjust the input topic in `inputs` and/or `export default function`.
//  3. Edit CONFIGS to add/remove/tweak output images.
//     → Copy an existing entry, spread DEFAULT_CONFIG, and change only
//       the fields you care about.
//  4. The output topic (/lidar_images) carries a message with an `images`
//     field — an array of image objects.  Access individual images via
//     Foxglove message paths such as `/lidar_images.images[0]`.
//     A downstream User Script can extract one entry and re-publish it
//     as a plain sensor_msgs/Image for the Raw Image panel.
//
// ============================================================

import { Input, Message } from "./types";

// ─── Topics ──────────────────────────────────────────────────────────────────
export const inputs = ["/multiscan/lidar_scan"];
export const output = "/lidar_images";

// ─── Encoding type ────────────────────────────────────────────────────────────
type Encoding = "mono16" | "rgb8" | "32FC1";

// ─── Default config ───────────────────────────────────────────────────────────
//
//  All CONFIGS entries are defined by spreading this object and overriding
//  only the fields you want to change.  Every field is documented here.
//
const DEFAULT_CONFIG = {
  /**
   * Human-readable name for this image output.
   * Stored in the `label` field of the corresponding images[] entry.
   */
  label: "default",

  /**
   * Output image width in pixels.
   * If 0, both width and height are dynamically derived from the data's
   * angular extents; otherwise height is derived from the FOV aspect ratio.
   */
  OUTPUT_WIDTH: 0,

  /**
   * Azimuth rotation applied before column-binning, in radians.
   * Positive = counter-clockwise when viewed from above (right-hand +z).
   * Use this to align "forward" with the image centre or edge.
   */
  AZIMUTH_OFFSET: 0,

  /**
   * Which point field maps to pixel brightness.
   *   "depth"       – geometric distance from the sensor origin (always
   *                   available; computed from x/y/z).
   *   anything else – read directly from a PointCloud2 field with that name
   *                   (e.g. "intensity", "reflectivity", "ring", "time").
   *                   Falls back to "depth" when the field is absent on the
   *                   cloud, logging a warning once per (label, field) pair.
   */
  VALUE_FIELD: "depth" as string,

  /**
   * Brightness scaling range for non-depth fields.
   * Pixel value 0 corresponds to VALUE_MIN; max (65535 or white) to VALUE_MAX.
   * Set VALUE_MIN === VALUE_MAX to auto-range from each frame's data instead.
   */
  VALUE_MIN: 0,
  VALUE_MAX: 0,

  /**
   * Power applied to the normalised [0, 1] brightness before encoding.
   * Values > 1 compress highlights; < 1 brighten dark regions.  1 = linear.
   */
  VALUE_RESCALE_POWER: 1,

  /**
   * Depth scaling range, used only when VALUE_FIELD === "depth".
   * Set MIN_DEPTH === MAX_DEPTH to auto-range from each frame's data.
   */
  MIN_DEPTH: 0,
  MAX_DEPTH: 0,

  /**
   * Pixel encoding of the output image.
   *   "mono16" – 16-bit greyscale, little-endian (best for downstream use;
   *              Foxglove auto-stretches the histogram for display).
   *   "rgb8"   – Jet false-colour (blue = low value, red = high value).
   *   "32FC1"  – Raw 32-bit float, no normalisation (value is stored as-is).
   */
  ENCODING: "32FC1" as Encoding,

  /**
   * Occlusion handling: when multiple points land on the same pixel, keep
   * the geometrically closest one (true) or the last one written (false).
   * true is higher quality; false is marginally faster.
   */
  KEEP_CLOSEST: true,

  /**
   * Vertical gap interpolation.
   * Sparse-layer LiDARs leave blank rows in the projected image.  This
   * setting controls the maximum run of consecutive empty rows that will be
   * linearly filled per column between the nearest filled row above and below.
   * Set to Infinity to always interpolate; set to 0 to disable entirely.
   */
  MAX_GAP_INTERPOLATION_ROWS: 0 as number,
};

/** The shape of every config entry (inferred from DEFAULT_CONFIG). */
type Config = typeof DEFAULT_CONFIG;

// ─── Output image list ────────────────────────────────────────────────────────
//
//  Each entry here produces one image in the output message's `images` array.
//  Only override the fields you want to change from DEFAULT_CONFIG — the rest
//  are inherited automatically via the spread operator.
//
//  Template for a new entry (copy → paste → edit):
//
//    {
//      ...DEFAULT_CONFIG,
//      label: "my_view",
//      VALUE_FIELD: "intensity",
//      ENCODING: "rgb8",
//    },
//
const CONFIGS: Config[] = [
  {
    ...DEFAULT_CONFIG,
    label: "rgb_depth",
    OUTPUT_WIDTH: 360,
    AZIMUTH_OFFSET: 1.57079632679,
    VALUE_FIELD: "depth",
    ENCODING: "rgb8",
    MIN_DEPTH: 0,
    MAX_DEPTH: 4,
    MAX_GAP_INTERPOLATION_ROWS: Infinity,
  },
  {
    ...DEFAULT_CONFIG,
    label: "raw_depth",
    OUTPUT_WIDTH: 360,
    AZIMUTH_OFFSET: 1.57079632679,
    VALUE_FIELD: "depth",
    ENCODING: "32FC1",
    MIN_DEPTH: 0,
    MAX_DEPTH: 0,
    MAX_GAP_INTERPOLATION_ROWS: Infinity,
  },
  {
    ...DEFAULT_CONFIG,
    label: "intensity",
    OUTPUT_WIDTH: 360,
    AZIMUTH_OFFSET: 1.57079632679,
    VALUE_FIELD: "intensity",
    VALUE_MIN: 30000,
    VALUE_MAX: 65535,
    VALUE_RESCALE_POWER: 2,
    ENCODING: "mono16",
    MAX_GAP_INTERPOLATION_ROWS: Infinity,
  },
];

// ─── Custom output message type ───────────────────────────────────────────────
//
//  Foxglove infers the output schema from this TypeScript type.
//  Each entry in `images` mirrors sensor_msgs/Image plus a `label` field.
//  Access individual images in Foxglove via message paths:
//    /lidar_images.images[0]   → first image (Entry 0 above)
//    /lidar_images.images[1]   → second image (Entry 1 above)
//
type ImageEntry = {
  /** Label from the config that produced this image. */
  label: string;
  height: number;
  width: number;
  encoding: string;
  is_bigendian: number;
  step: number;
  data: Uint8Array;
};

type MultiImageMessage = {
  header: Message<"std_msgs/Header">;
  images: ImageEntry[];
};

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

/** Encoding string → bytes per pixel. */
const bppMap = new Map<string, number>([
  ["mono16", 2],
  ["rgb8", 3],
  ["32FC1", 4],
]);

/**
 * Tracks (label:fieldName) pairs for which a missing-field warning has already
 * been emitted, so the log isn't spammed on every frame.
 */
const warnedMissingFields = new Set<string>();

// ─── Helper: read one scalar from a DataView ──────────────────────────────────
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
 * between the nearest filled row above and below.  Blank runs longer than
 * `maxGap` rows are left untouched.  Edge runs (no filled row on one side)
 * are also left untouched.
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

// ─── Shared parse: extract valid x/y/z once per frame ────────────────────────
//
//  Rather than scanning the entire PointCloud2 buffer once per config entry,
//  we extract x, y, z (and the byte offset of each valid point) in a single
//  shared pass.  Per-config value fields are then looked up cheaply using
//  the stored offsets.
//
type ParsedPoints = {
  xb: Float32Array; // x coordinate of each valid point
  yb: Float32Array; // y coordinate
  zb: Float32Array; // z coordinate
  bases: Uint32Array; // byte offset of each valid point in the cloud buffer
  count: number; // number of valid points stored
};

function parseXYZ(
  view: DataView,
  numPoints: number,
  pointStep: number,
  xf: { offset: number; datatype: number },
  yf: { offset: number; datatype: number },
  zf: { offset: number; datatype: number },
  le: boolean,
): ParsedPoints {
  const xb = new Float32Array(numPoints);
  const yb = new Float32Array(numPoints);
  const zb = new Float32Array(numPoints);
  const bases = new Uint32Array(numPoints);
  let count = 0;

  for (let i = 0; i < numPoints; i++) {
    const base = i * pointStep;
    const x = readScalar(view, base + xf.offset, xf.datatype, le);
    const y = readScalar(view, base + yf.offset, yf.datatype, le);
    const z = readScalar(view, base + zf.offset, zf.datatype, le);
    if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;
    xb[count] = x;
    yb[count] = y;
    zb[count] = z;
    bases[count] = base;
    count++;
  }

  return { xb, yb, zb, bases, count };
}

// ─── Per-config rendering ─────────────────────────────────────────────────────
function renderImage(
  cfg: Config,
  fieldMap: Map<string, { offset: number; datatype: number }>,
  pts: ParsedPoints,
  view: DataView,
  le: boolean,
): ImageEntry | undefined {
  const { xb, yb, zb, bases, count } = pts;

  // ── Resolve value field ─────────────────────────────────────────────────
  const useDepthAsValue = cfg.VALUE_FIELD === "depth";
  const vf = useDepthAsValue ? null : (fieldMap.get(cfg.VALUE_FIELD) ?? null);

  if (!useDepthAsValue && vf == null) {
    const warnKey = `${cfg.label}:${cfg.VALUE_FIELD}`;
    if (!warnedMissingFields.has(warnKey)) {
      log(
        `[lidar_images "${cfg.label}"] Field "${cfg.VALUE_FIELD}" not found` +
          ` (available: ${[...fieldMap.keys()].join(", ")}). Falling back to depth.`,
      );
      warnedMissingFields.add(warnKey);
    }
  }
  const useVF = !useDepthAsValue && vf != null;

  const autoDepthRange = cfg.MIN_DEPTH === cfg.MAX_DEPTH;
  const autoValueRange = cfg.VALUE_MIN === cfg.VALUE_MAX;

  // ── Pass 1 – compute spherical coords and per-point values ──────────────
  const azBuf = new Float32Array(count);
  const elBuf = new Float32Array(count);
  const depBuf = new Float32Array(count);
  const valBuf = new Float32Array(count);
  let valid = 0;

  let minAz = Infinity,
    maxAz = -Infinity;
  let minEl = Infinity,
    maxEl = -Infinity;
  let minDep = Infinity,
    maxDep = -Infinity;
  let minVal = Infinity,
    maxVal = -Infinity;

  for (let i = 0; i < count; i++) {
    const x = xb[i],
      y = yb[i],
      z = zb[i];
    const xyDist = Math.sqrt(x * x + y * y);
    const depth = Math.sqrt(xyDist * xyDist + z * z);

    // Spherical coordinates with configurable azimuth offset, wrapped to [-π, π)
    let az = Math.atan2(y, x) + cfg.AZIMUTH_OFFSET;
    az =
      ((((az + Math.PI) % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI)) -
      Math.PI;
    const el = Math.atan2(z, xyDist);

    // Displayed value: depth itself, or a named field from the point bytes
    let val = depth;
    if (useVF && vf != null) {
      val = readScalar(view, bases[i] + vf.offset, vf.datatype, le);
      if (!isFinite(val)) continue;
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

  // ── Resolve scaling ranges ───────────────────────────────────────────────
  const depthMin = autoDepthRange ? minDep : cfg.MIN_DEPTH;
  const depthMax = autoDepthRange ? maxDep : cfg.MAX_DEPTH;

  const valueMin = useDepthAsValue
    ? depthMin
    : autoValueRange
      ? minVal
      : cfg.VALUE_MIN;
  const valueMax = useDepthAsValue
    ? depthMax
    : autoValueRange
      ? maxVal
      : cfg.VALUE_MAX;
  const valueRange = Math.max(valueMax - valueMin, 1e-6);

  // ── Determine image dimensions ───────────────────────────────────────────
  const azOrigin = minAz;
  const azRange = Math.max(maxAz - minAz, 1e-6);
  const elOrigin = minEl;
  const elRange = Math.max(maxEl - minEl, 1e-6);
  const aspect = azRange / elRange;

  let outW: number, outH: number;
  if (cfg.OUTPUT_WIDTH > 0) {
    outW = cfg.OUTPUT_WIDTH;
    outH = Math.max(1, Math.floor(outW / aspect));
  } else {
    outH = Math.max(16, Math.min(1024, Math.round(Math.sqrt(valid / aspect))));
    outW = Math.max(16, Math.min(4096, Math.round(outH * aspect)));
  }

  // ── Allocate image and value buffers ────────────────────────────────────
  const bytesPerPixel = bppMap.get(cfg.ENCODING) as number;
  const rowStep = outW * bytesPerPixel;
  const isRaw = cfg.ENCODING === "32FC1";
  const imgBuf = isRaw
    ? new Float32Array(outH * outW)
    : new Uint8Array(outH * rowStep);

  // valuePx holds the "winning" value for each pixel; NaN = empty.
  const valuePx = new Float32Array(outW * outH).fill(NaN);
  // depthKeyPx tracks the shallowest depth seen per pixel (occlusion key).
  const depthKeyPx = cfg.KEEP_CLOSEST
    ? new Float32Array(outW * outH).fill(Infinity)
    : null;

  // ── Pass 2 – project points onto the value buffer ────────────────────────
  for (let i = 0; i < valid; i++) {
    const u = (azBuf[i] - azOrigin) / azRange;
    const v = (elBuf[i] - elOrigin) / elRange;
    if (u < 0 || u >= 1 || v < 0 || v >= 1) continue;

    // Flip horizontally so the image matches left/right physical sense.
    // Flip vertically so higher elevation → lower row index (top of image).
    const col = outW - 1 - Math.floor(u * outW);
    const row = Math.floor((1.0 - v) * outH);
    if (col < 0 || col >= outW || row < 0 || row >= outH) continue;

    const pixIdx = row * outW + col;
    if (depthKeyPx != null) {
      const dep = depBuf[i];
      if (dep >= depthKeyPx[pixIdx]) continue; // another point is closer
      depthKeyPx[pixIdx] = dep;
    }
    valuePx[pixIdx] = valBuf[i];
  }

  // ── Pass 3 – bridge blank rows left by sparse vertical layers ────────────
  if (cfg.MAX_GAP_INTERPOLATION_ROWS > 0) {
    interpolateColumnGaps(valuePx, outW, outH, cfg.MAX_GAP_INTERPOLATION_ROWS);
  }

  // ── Pass 4 – encode value buffer into the output pixel format ────────────
  for (let pixIdx = 0; pixIdx < outW * outH; pixIdx++) {
    const val = valuePx[pixIdx];

    if (isRaw) {
      (imgBuf as Float32Array)[pixIdx] = isNaN(val) ? 0 : val;
      continue;
    }

    if (isNaN(val)) continue; // pixel stays 0 (zero-initialised buffer)

    // Normalise → apply rescale power → encode
    const t = Math.pow(
      Math.max(0.0, Math.min(1.0, (val - valueMin) / valueRange)),
      cfg.VALUE_RESCALE_POWER,
    );

    if (cfg.ENCODING === "rgb8") {
      const [r, g, b] = jet(t);
      const byteIdx = pixIdx * 3;
      (imgBuf as Uint8Array)[byteIdx] = r;
      (imgBuf as Uint8Array)[byteIdx + 1] = g;
      (imgBuf as Uint8Array)[byteIdx + 2] = b;
    } else {
      // mono16, little-endian: 0x0000 = valueMin, 0xFFFF = valueMax
      const val16 = Math.round(t * 65535);
      const byteIdx = pixIdx * 2;
      (imgBuf as Uint8Array)[byteIdx] = val16 & 0xff;
      (imgBuf as Uint8Array)[byteIdx + 1] = (val16 >> 8) & 0xff;
    }
  }

  return {
    label: cfg.label,
    height: outH,
    width: outW,
    encoding: cfg.ENCODING,
    is_bigendian: 0,
    step: rowStep,
    data: new Uint8Array(imgBuf.buffer),
  };
}

// ─── Main entry point ─────────────────────────────────────────────────────────
export default function script(
  event: Input<"/multiscan/lidar_scan">,
): MultiImageMessage | undefined {
  const cloud = event.message as Message<"sensor_msgs/PointCloud2">;

  // ── Build field lookup map ──────────────────────────────────────────────
  const fieldMap = new Map<string, { offset: number; datatype: number }>();
  for (const f of cloud.fields) {
    fieldMap.set(f.name, { offset: f.offset, datatype: f.datatype });
  }

  const xf = fieldMap.get("x");
  const yf = fieldMap.get("y");
  const zf = fieldMap.get("z");
  if (xf == null || yf == null || zf == null) return undefined;

  const { point_step, data, is_bigendian, header } = cloud;
  const le = !is_bigendian;
  const view = new DataView(data.buffer, data.byteOffset, data.byteLength);
  const numPoints = Math.floor(data.byteLength / point_step);
  if (numPoints === 0) return undefined;

  // ── Shared pass: parse x/y/z once for all valid points ─────────────────
  const pts = parseXYZ(view, numPoints, point_step, xf, yf, zf, le);
  if (pts.count === 0) return undefined;

  // ── Render one image per config entry ───────────────────────────────────
  const images: ImageEntry[] = [];
  for (const cfg of CONFIGS) {
    const img = renderImage(cfg, fieldMap, pts, view, le);
    if (img != null) images.push(img);
  }

  if (images.length === 0) return undefined;

  return { header, images };
}
