import BufferView from "./BufferView";
import { decodeVariant } from "./DecodingUtils";

export enum FieldType {
  UNKNOWN = 0,
  INT8 = 1,
  UINT8 = 2,
  INT16 = 3,
  UINT16 = 4,
  INT32 = 5,
  UINT32 = 6,
  FLOAT32 = 7,
  FLOAT64 = 8,
}

export interface FieldDecoder {
  decode(encodedView: BufferView, outputDataView: DataView, pointOffset: number): void;
}

export class FieldDecoderCopy implements FieldDecoder {
  private fieldOffset: number;
  private fieldSize: number;

  constructor(field: { offset: number }, fieldSize: number) {
    this.fieldOffset = field.offset;
    this.fieldSize = fieldSize;
  }

  decode(encodedView: BufferView, outputDataView: DataView, pointOffset: number): void {
    const dataToCopy = encodedView.getData().subarray(0, this.fieldSize);
    const outputBuffer = new Uint8Array(outputDataView.buffer);
    outputBuffer.set(dataToCopy, pointOffset + this.fieldOffset);
    encodedView.trimFront(this.fieldSize);
  }
}

export class FieldDecoderInt implements FieldDecoder {
  private fieldOffset: number;
  private prevValue = 0n;
  private setter: (offset: number, value: number, littleEndian?: boolean) => void;

  constructor(field: { offset: number }, setter: (offset: number, value: number, littleEndian?: boolean) => void) {
    this.fieldOffset = field.offset;
    this.setter = setter;
  }

  decode(encodedView: BufferView, outputDataView: DataView, pointOffset: number): void {
    const delta = decodeVariant(encodedView);
    const value = this.prevValue + delta;
    this.setter.call(outputDataView, pointOffset + this.fieldOffset, Number(value), true);
    this.prevValue = value;
  }
}

export class FieldDecoderFloatLossy implements FieldDecoder {
  private fieldOffset: number;
  private prevValue = 0n;
  private resolution: number;
  private setter: (offset: number, value: number, littleEndian?: boolean) => void;

  constructor(
    field: { offset: number; resolution?: number },
    setter: (offset: number, value: number, littleEndian?: boolean) => void,
  ) {
    this.fieldOffset = field.offset;
    this.resolution = field.resolution!;
    this.setter = setter;
  }

  decode(encodedView: BufferView, outputDataView: DataView, pointOffset: number): void {
    const delta = decodeVariant(encodedView);
    const value_int = this.prevValue + delta;
    const value_real = Number(value_int) * this.resolution;
    this.setter.call(outputDataView, pointOffset + this.fieldOffset, value_real, true);
    this.prevValue = value_int;
  }
}

export class FieldDecoderSkip implements FieldDecoder {
  private advanceFn: (view: BufferView) => void;

  constructor(field: { datatype: FieldType; resolution?: number }) {
    switch (field.datatype) {
      case FieldType.FLOAT32:
        this.advanceFn = field.resolution
          ? (view) => {
              decodeVariant(view);
            }
          : (view) => {
              view.trimFront(4);
            };
        break;
      case FieldType.FLOAT64:
        this.advanceFn = field.resolution
          ? (view) => {
              decodeVariant(view);
            }
          : (view) => {
              view.trimFront(8);
            };
        break;
      case FieldType.INT8:
      case FieldType.UINT8:
        this.advanceFn = (view) => {
          view.trimFront(1);
        };
        break;
      case FieldType.INT16:
      case FieldType.UINT16:
      case FieldType.INT32:
      case FieldType.UINT32:
        this.advanceFn = (view) => {
          decodeVariant(view);
        };
        break;
      default:
        throw new Error(`Unsupported field type for skip: ${field.datatype}`);
    }
  }

  decode(encodedView: BufferView, _outputDataView: DataView, _pointOffset: number): void {
    this.advanceFn(encodedView);
  }
}
