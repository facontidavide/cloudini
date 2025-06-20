class BufferView {
  private data: Uint8Array;
  private offset: number;

  constructor(data: Uint8Array) {
    this.data = data;
    this.offset = 0;
  }

  // Get current data pointer
  getData(): Uint8Array {
    return this.data.subarray(this.offset);
  }

  // Advance the buffer by n bytes
  trimFront(n: number) {
    this.offset += n;
  }

  // Get current size
  size(): number {
    return this.data.length - this.offset;
  }
}

export default BufferView;
