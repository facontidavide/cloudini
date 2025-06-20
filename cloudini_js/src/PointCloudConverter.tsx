import * as fzstd from "fzstd";

import { CompressedPointCloud } from "./Schemas";
import { decodeHeader } from "./DecodeHeader";
import BufferView from "./BufferView";
import {
  FieldDecoder,
  FieldDecoderCopy,
  FieldDecoderFloatLossy,
  FieldDecoderInt,
  FieldDecoderSkip,
} from "./FieldDecoders";
import { FieldType } from "./FieldDecoders";

// If a filed value is equal to this once one it means that it was encoded, but that we do not want to decode it
const K_DECODE_BUT_SKIP_STORE = 2 ** 32 - 1;

export const convertPointCloud = (cloud: CompressedPointCloud) => {
  let errorString = "";

  try {
    // First decode the header
    const compressed = new Uint8Array(cloud.compressed_data);
    const { headerSize, header } = decodeHeader(compressed);

    // The rest of the data after the header is ZSTD compressed
    const compressedData = compressed.subarray(headerSize);

    // First decode back from zstd
    const firstStageEncodedData = fzstd.decompress(compressedData);

    const finalData = new Uint8Array(header.width * header.height * header.pointStep);
    const finalDataView = new DataView(finalData.buffer);
    const encodedView = new BufferView(firstStageEncodedData);

    const decoders: FieldDecoder[] = [];
    for (const field of header.fields) {
      if (field.offset === K_DECODE_BUT_SKIP_STORE) {
        decoders.push(new FieldDecoderSkip(field));
        continue;
      }

      switch (field.datatype) {
        case FieldType.FLOAT32:
          decoders.push(
            field.resolution
              ? new FieldDecoderFloatLossy(field, finalDataView.setFloat32.bind(finalDataView))
              : new FieldDecoderCopy(field, 4)
          );
          break;
        case FieldType.FLOAT64:
          decoders.push(
            field.resolution
              ? new FieldDecoderFloatLossy(field, finalDataView.setFloat64.bind(finalDataView))
              : new FieldDecoderCopy(field, 8)
          );
          break;
        case FieldType.INT8:
          decoders.push(new FieldDecoderCopy(field, 1));
          break;
        case FieldType.UINT8:
          decoders.push(new FieldDecoderCopy(field, 1));
          break;
        case FieldType.INT16:
          decoders.push(new FieldDecoderInt(field, finalDataView.setInt16.bind(finalDataView)));
          break;
        case FieldType.UINT16:
          decoders.push(new FieldDecoderInt(field, finalDataView.setUint16.bind(finalDataView)));
          break;
        case FieldType.INT32:
          decoders.push(new FieldDecoderInt(field, finalDataView.setInt32.bind(finalDataView)));
          break;
        case FieldType.UINT32:
          decoders.push(new FieldDecoderInt(field, finalDataView.setUint32.bind(finalDataView)));
          break;
        default:
          throw new Error(`Unsupported field type: ${field.datatype}`);
      }
    }

    const numPoints = header.width * header.height;

    for (let i = 0; i < numPoints; i++) {
      const pointOffset = i * header.pointStep;
      for (const decoder of decoders) {
        decoder.decode(encodedView, finalDataView, pointOffset);
      }
    }

    // Create the decoded point cloud message
    const decodedMsg = {
      header: {
        frame_id: cloud.header.frame_id,
        stamp: cloud.header.stamp,
      },
      height: header.height,
      width: header.width,
      fields: header.fields,
      is_bigendian: false,
      point_step: header.pointStep,
      row_step: header.pointStep * header.width,
      is_dense: cloud.is_dense,
      data: finalData,
    };

    return decodedMsg;
  } catch (error) {
    errorString = `Error decompressing point cloud: ${error}`;
    console.error(errorString);
    return {};
  }
};
