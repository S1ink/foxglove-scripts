import { Message } from "./types"

export const inputs = ["/foxglove/lidar_image/preproc"];
export const output = "/foxglove/lidar_image/image";


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


export default function script(event: any): Message<"sensor_msgs/Image"> | undefined {
    const preproc = event.message as CloudPreproc;

    return;
}
