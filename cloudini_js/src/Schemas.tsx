type Header = {
  frame_id: string;
  stamp: number;
};

enum PointFieldDatatype {
  INT8 = 1,
  UINT8 = 2,
  INT16 = 3,
  UINT16 = 4,
  INT32 = 5,
  UINT32 = 6,
  FLOAT32 = 7,
  FLOAT64 = 8,
}

type PointField = {
  name: string; //  Name of field
  offset: number; // Offset from start of point struct
  datatype: number | PointFieldDatatype; // Datatype enumeration, see above
  count: number; // How many elements in the field
};

export type CompressedPointCloud = {
  header: Header;
  height: number;
  width: number;
  fields: PointField[];
  is_bigendian: boolean;
  point_step: number;
  row_step: number;
  is_dense: boolean;
  compressed_data: Uint8Array;
};
