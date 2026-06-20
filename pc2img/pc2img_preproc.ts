import { Message } from "./types";

export const inputs = ["/multiscan/lidar_scan"];
export const output = "/foxglove/lidar_image/preproc";


type F32ArrayContainer = Message<"sensor_msgs/LaserEcho">;

type FieldValues = {
  name: string;
  datatype: number;
  count: number;
  data: F32ArrayContainer;
};

type CloudPreproc = {
  header: Message<"std_msgs/Header">;
  num_points: number;

  azimuths: F32ArrayContainer;
  elevations: F32ArrayContainer;
  ranges: F32ArrayContainer;

  min_azimuth: number;
  max_azimuth: number;
  min_elevation: number;
  max_elevation: number;
  min_range: number;
  max_range: number;

  values: FieldValues[]
};

const DT = {
  INT8: 1,
  UINT8: 2,
  INT16: 3,
  UINT16: 4,
  INT32: 5,
  UINT32: 6,
  FLOAT32: 7,
  FLOAT64: 8,
  INT64: 9,
  UINT64: 10,
  BOOL: 11,
} as const;

const DT_SIZE = [0, 1, 1, 2, 2, 4, 4, 4, 8, 8, 8, 1] as const;

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
    case DT.INT64:
      return Number(view.getBigInt64(byteOffset, le));
    case DT.UINT64:
      return Number(view.getBigInt64(byteOffset, le));
    default:
      return NaN;
  }
}

export default function script(event: any): CloudPreproc | undefined {
  const cloud = event.message as Message<"sensor_msgs/PointCloud2">;

  const fieldMap = new Map<string, { offset: number; datatype: number }>();
  for (const f of cloud.fields) {
    fieldMap.set(f.name, { offset: f.offset, datatype: f.datatype });
  }

  const xf = fieldMap.get("x");
  const yf = fieldMap.get("y");
  const zf = fieldMap.get("z");
  if (xf == null || yf == null || zf == null) return undefined;

  const data = cloud.data;
  const pointStep = cloud.point_step;
  const le = !cloud.is_bigendian;
  const view = new DataView(
    data.buffer,
    data.byteOffset,
    data.byteLength,
  );
  const cloudNumPoints = Math.floor(data.byteLength / pointStep);

  const azimuths = new Float32Array(cloudNumPoints);
  const elevations = new Float32Array(cloudNumPoints);
  const ranges = new Float32Array(cloudNumPoints);
  const bases = new Uint32Array(cloudNumPoints);
  let num_points = 0;

  let min_azimuth = Infinity,
    max_azimuth = -Infinity;
  let min_elevation = Infinity,
    max_elevation = -Infinity;
  let min_range = Infinity,
    max_range = -Infinity;

  // read all xyz, convert to polar, keep track of valid indices
  for (let i = 0; i < cloudNumPoints; i++) {
    const base = i * pointStep;
    const x = readScalar(view, base + xf.offset, xf.datatype, le);
    const y = readScalar(view, base + yf.offset, yf.datatype, le);
    const z = readScalar(view, base + zf.offset, zf.datatype, le);

    if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;

    const xyDistSqrd = x * x + y * y;
    const range = Math.sqrt(xyDistSqrd + z * z);

    if (range < 1e-4) continue;

    const azimuth =
      ((((Math.atan2(y, x) + Math.PI) % (2 * Math.PI)) + 2 * Math.PI) %
        (2 * Math.PI)) -
      Math.PI;
    const elevation = Math.atan2(z, Math.sqrt(xyDistSqrd));

    if (azimuth < min_azimuth) min_azimuth = azimuth;
    if (azimuth > max_azimuth) max_azimuth = azimuth;
    if (elevation < min_elevation) min_elevation = elevation;
    if (elevation > max_elevation) max_elevation = elevation;
    if (range < min_range) min_range = range;
    if (range > max_range) max_range = range;

    azimuths[num_points] = azimuth;
    elevations[num_points] = elevation;
    ranges[num_points] = range;
    bases[num_points] = base;
    num_points++;
  }

  // extract all values for each additional field
  const ignoreFields = new Set<string>([
    "x",
    "y",
    "z",
    "azimuth",
    "elevation",
    "range",
    "depth",
  ]);

  const values: FieldValues[] = [];
  fieldMap.forEach((value, key, map) => {
    if(ignoreFields.has(key)) return;

    const datatype = value.datatype;
    if (datatype > DT_SIZE.length) return;

    const valSize = DT_SIZE[datatype];
    const numF32s = Math.ceil((num_points * valSize) / Float32Array.BYTES_PER_ELEMENT);
    const bytes = new Uint8Array(numF32s * Float32Array.BYTES_PER_ELEMENT);
    for (let i = 0; i < num_points; i++) {
      const off = bases[i] + value.offset;
      bytes.set(data.subarray(off, off + valSize), i * valSize);
    }

    values.push({
      name: key,
      datatype,
      count: num_points,
      data: { echoes: new Float32Array(bytes.buffer, 0, numF32s) },
    } as FieldValues);
  });

  return {
    header: cloud.header,
    num_points,
    azimuths: { echoes: azimuths.subarray(0, num_points) },
    elevations: { echoes: azimuths.subarray(0, num_points) },
    ranges: { echoes: azimuths.subarray(0, num_points) },
    min_azimuth,
    max_azimuth,
    min_elevation,
    max_elevation,
    min_range,
    max_range,
    values
  };
}
