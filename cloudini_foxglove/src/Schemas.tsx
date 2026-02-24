// ============================================
// Volirot types (for CloudiniCompressedPointCloud input)
// ============================================

// Timestamp type (nanoseconds since epoch)
type VolirotTimestamp = {
  timestamp_ns: number;
};

// Vector3d type
type Vector3d = {
  values: number[]; // [x, y, z]
};

// Quaterniond type
type Quaterniond = {
  values_xyzw: number[]; // [x, y, z, w]
};

// Transform type
type VolirotTransform = {
  translation: Vector3d;
  rotation: Quaterniond;
};

// ============================================
// Point cloud with Cloudini compression applied to point data.
// ============================================
export type CloudiniCompressedPointCloud = {
  // Timestamp of the point cloud.
  timestamp: VolirotTimestamp;
  // Frame of reference (coordinate frame ID).
  frame_id: string;
  // Optional pose of the point cloud origin relative to the frame.
  pose?: VolirotTransform;
  // Compressed point data.
  compressed_data: Uint8Array;
};
