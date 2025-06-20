import BufferView from "./BufferView";

export function decodeVariant(view: BufferView): bigint {
  let result = 0n;
  let shift = 0n;
  const data = view.getData();
  let bytesRead = 0;

  for (let i = 0; i < data.length; i++) {
    bytesRead++;
    const byte = data[i]!;
    result |= BigInt(byte & 0x7f) << shift;
    shift += 7n;
    if ((byte & 0x80) === 0) {
      break;
    }
  }

  view.trimFront(bytesRead);
  // we have reserved the value 0 for NaN
  result--;

  // Perform zigzag decoding
  // See https://stackoverflow.com/questions/2210923/zig-zag-decoding
  const value = (result >> 1n) ^ -(result & 1n);
  return value;
}

export function decode<T>(view: BufferView, size: number): T {
  const buffer = view.getData();
  const dataView = new DataView(buffer.buffer, buffer.byteOffset, size);
  let result: any;

  switch (size) {
    case 1:
      result = dataView.getUint8(0);
      break;
    case 2:
      result = dataView.getUint16(0, true); // little endian
      break;
    case 4:
      if (buffer.byteOffset % 4 === 0) {
        // If aligned, use the faster Uint32Array
        result = new Uint32Array(buffer.buffer, buffer.byteOffset, 1)[0];
      } else {
        result = dataView.getUint32(0, true); // little endian
      }
      break;
    default:
      throw new Error(`Unsupported decode size: ${size}`);
  }

  view.trimFront(size);
  return result as T;
}

export function decodeString(view: BufferView): string {
  const len = decode<number>(view, 2); // uint16_t length
  const strData = view.getData().subarray(0, len);
  const decoder = new TextDecoder();
  const str = decoder.decode(strData);
  view.trimFront(len);
  return str;
}

export function decodeFloat(view: BufferView): number | undefined {
  const buffer = view.getData();
  const dataView = new DataView(buffer.buffer, buffer.byteOffset, 4);
  const value = dataView.getFloat32(0, true); // true for little-endian
  view.trimFront(4);
  return value;
}
