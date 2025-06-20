import BufferView from "./BufferView";

const MAGIC_HEADER = "CLOUDINI_V01";
import { decode, decodeFloat, decodeString } from "./DecodingUtils";

export function decodeHeader(data: Uint8Array) {
  const view = new BufferView(data);

  // Check magic header, this indicates that the header is present at the beginning of the buffer
  const magicHeader = new TextDecoder().decode(view.getData().subarray(0, MAGIC_HEADER.length));
  if (magicHeader !== MAGIC_HEADER) {
    throw new Error(`Invalid magic header. Expected ${MAGIC_HEADER}, got: ${magicHeader}`);
  }
  view.trimFront(MAGIC_HEADER.length);

  const width = decode<number>(view, 4);
  const height = decode<number>(view, 4);
  const pointStep = decode<number>(view, 4);
  const encodingOpt = decode<number>(view, 1);
  const compressionOpt = decode<number>(view, 1);
  const fieldsCount = decode<number>(view, 2);

  // Decode fields array
  const fields = [];
  for (let i = 0; i < fieldsCount; i++) {
    const name = decodeString(view);
    const offset = decode<number>(view, 4); // uint32_t
    const type = decode<number>(view, 1); // uint8_t (FieldType)
    const resolution = decodeFloat(view); // float

    fields.push({
      name,
      offset,
      datatype: type,
      count: 1,
      resolution: resolution! > 0 ? resolution : undefined,
    });
  }

  // Calculate header size for later use
  const headerSize = data.length - view.size();

  return {
    headerSize,
    header: {
      width,
      height,
      pointStep,
      encodingOpt,
      compressionOpt,
      fields,
    },
  };
}
