package org.bytedeco.javacpp.indexer;

import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.Pointer;

public class FloatRawIndexer extends FloatIndexer {
    protected static final Raw RAW = Raw.getInstance();
    final long base;
    protected FloatPointer pointer;
    final long size;

    public FloatRawIndexer(FloatPointer pointer2) {
        this(pointer2, new long[]{pointer2.limit() - pointer2.position()}, ONE_STRIDE);
    }

    public FloatRawIndexer(FloatPointer pointer2, long[] sizes, long[] strides) {
        super(sizes, strides);
        this.pointer = pointer2;
        this.base = pointer2.address() + (pointer2.position() * 4);
        this.size = pointer2.limit() - pointer2.position();
    }

    public Pointer pointer() {
        return this.pointer;
    }

    public float get(long i) {
        return RAW.getFloat(this.base + (checkIndex(i, this.size) * 4));
    }

    public FloatIndexer get(long i, float[] f, int offset, int length) {
        for (int n = 0; n < length; n++) {
            f[offset + n] = get((this.strides[0] * i) + ((long) n));
        }
        return this;
    }

    public float get(long i, long j) {
        return get((this.strides[0] * i) + j);
    }

    public FloatIndexer get(long i, long j, float[] f, int offset, int length) {
        for (int n = 0; n < length; n++) {
            f[offset + n] = get((this.strides[0] * i) + (this.strides[1] * j) + ((long) n));
        }
        return this;
    }

    public float get(long i, long j, long k) {
        return get((this.strides[0] * i) + (this.strides[1] * j) + k);
    }

    public float get(long... indices) {
        return get(index(indices));
    }

    public FloatIndexer get(long[] indices, float[] f, int offset, int length) {
        for (int n = 0; n < length; n++) {
            f[offset + n] = get(index(indices) + ((long) n));
        }
        return this;
    }

    public FloatIndexer put(long i, float f) {
        RAW.putFloat(this.base + (checkIndex(i, this.size) * 4), f);
        return this;
    }

    public FloatIndexer put(long i, float[] f, int offset, int length) {
        for (int n = 0; n < length; n++) {
            put((this.strides[0] * i) + ((long) n), f[offset + n]);
        }
        return this;
    }

    public FloatIndexer put(long i, long j, float f) {
        put((this.strides[0] * i) + j, f);
        return this;
    }

    public FloatIndexer put(long i, long j, float[] f, int offset, int length) {
        for (int n = 0; n < length; n++) {
            put((this.strides[0] * i) + (this.strides[1] * j) + ((long) n), f[offset + n]);
        }
        return this;
    }

    public FloatIndexer put(long i, long j, long k, float f) {
        put((this.strides[0] * i) + (this.strides[1] * j) + k, f);
        return this;
    }

    public FloatIndexer put(long[] indices, float f) {
        put(index(indices), f);
        return this;
    }

    public FloatIndexer put(long[] indices, float[] f, int offset, int length) {
        for (int n = 0; n < length; n++) {
            put(index(indices) + ((long) n), f[offset + n]);
        }
        return this;
    }

    public void release() {
        this.pointer = null;
    }
}
