package org.bytedeco.javacpp.indexer;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.Pointer;

public class ByteRawIndexer extends ByteIndexer {
    protected static final Raw RAW = Raw.getInstance();
    final long base;
    protected BytePointer pointer;
    final long size;

    public ByteRawIndexer(BytePointer pointer2) {
        this(pointer2, new long[]{pointer2.limit() - pointer2.position()}, ONE_STRIDE);
    }

    public ByteRawIndexer(BytePointer pointer2, long[] sizes, long[] strides) {
        super(sizes, strides);
        this.pointer = pointer2;
        this.base = pointer2.address() + pointer2.position();
        this.size = pointer2.limit() - pointer2.position();
    }

    public Pointer pointer() {
        return this.pointer;
    }

    public byte get(long i) {
        return RAW.getByte(this.base + checkIndex(i, this.size));
    }

    public ByteIndexer get(long i, byte[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            b[offset + n] = get((this.strides[0] * i) + ((long) n));
        }
        return this;
    }

    public byte get(long i, long j) {
        return get((this.strides[0] * i) + ((long) ((int) j)));
    }

    public ByteIndexer get(long i, long j, byte[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            b[offset + n] = get((this.strides[0] * i) + (this.strides[1] * j) + ((long) n));
        }
        return this;
    }

    public byte get(long i, long j, long k) {
        return get((this.strides[0] * i) + (this.strides[1] * j) + k);
    }

    public byte get(long... indices) {
        return get(index(indices));
    }

    public ByteIndexer get(long[] indices, byte[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            b[offset + n] = get(index(indices) + ((long) n));
        }
        return this;
    }

    public ByteIndexer put(long i, byte b) {
        RAW.putByte(this.base + checkIndex(i, this.size), b);
        return this;
    }

    public ByteIndexer put(long i, byte[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            put((this.strides[0] * i) + ((long) n), b[offset + n]);
        }
        return this;
    }

    public ByteIndexer put(long i, long j, byte b) {
        put((this.strides[0] * i) + j, b);
        return this;
    }

    public ByteIndexer put(long i, long j, byte[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            put((this.strides[0] * i) + (this.strides[1] * j) + ((long) n), b[offset + n]);
        }
        return this;
    }

    public ByteIndexer put(long i, long j, long k, byte b) {
        put((this.strides[0] * i) + (this.strides[1] * j) + k, b);
        return this;
    }

    public ByteIndexer put(long[] indices, byte b) {
        put(index(indices), b);
        return this;
    }

    public ByteIndexer put(long[] indices, byte[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            put(index(indices) + ((long) n), b[offset + n]);
        }
        return this;
    }

    public short getShort(long i) {
        return RAW.getShort(this.base + checkIndex(i, this.size - 1));
    }

    public ByteIndexer putShort(long i, short s) {
        RAW.putShort(this.base + checkIndex(i, this.size - 1), s);
        return this;
    }

    public int getInt(long i) {
        return RAW.getInt(this.base + checkIndex(i, this.size - 3));
    }

    public ByteIndexer putInt(long i, int j) {
        RAW.putInt(this.base + checkIndex(i, this.size - 3), j);
        return this;
    }

    public long getLong(long i) {
        return RAW.getLong(this.base + checkIndex(i, this.size - 7));
    }

    public ByteIndexer putLong(long i, long j) {
        RAW.putLong(this.base + checkIndex(i, this.size - 7), j);
        return this;
    }

    public float getFloat(long i) {
        return RAW.getFloat(this.base + checkIndex(i, this.size - 3));
    }

    public ByteIndexer putFloat(long i, float f) {
        RAW.putFloat(this.base + checkIndex(i, this.size - 3), f);
        return this;
    }

    public double getDouble(long i) {
        return RAW.getDouble(this.base + checkIndex(i, this.size - 7));
    }

    public ByteIndexer putDouble(long i, double d) {
        RAW.putDouble(this.base + checkIndex(i, this.size - 7), d);
        return this;
    }

    public char getChar(long i) {
        return RAW.getChar(this.base + checkIndex(i, this.size - 1));
    }

    public ByteIndexer putChar(long i, char c) {
        RAW.putChar(this.base + checkIndex(i, this.size - 1), c);
        return this;
    }

    public void release() {
        this.pointer = null;
    }
}
