package org.bytedeco.javacpp.indexer;

import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacpp.ShortPointer;

public class Bfloat16RawIndexer extends Bfloat16Indexer {
    protected static final Raw RAW = Raw.getInstance();
    final long base;
    protected ShortPointer pointer;
    final long size;

    public Bfloat16RawIndexer(ShortPointer pointer2) {
        this(pointer2, new long[]{pointer2.limit() - pointer2.position()}, ONE_STRIDE);
    }

    public Bfloat16RawIndexer(ShortPointer pointer2, long[] sizes, long[] strides) {
        super(sizes, strides);
        this.pointer = pointer2;
        this.base = pointer2.address() + (pointer2.position() * 2);
        this.size = pointer2.limit() - pointer2.position();
    }

    public Pointer pointer() {
        return this.pointer;
    }

    public float get(long i) {
        return toFloat(RAW.getShort(this.base + (checkIndex(i, this.size) * 2)));
    }

    public Bfloat16Indexer get(long i, float[] h, int offset, int length) {
        for (int n = 0; n < length; n++) {
            h[offset + n] = get((this.strides[0] * i) + ((long) n));
        }
        return this;
    }

    public float get(long i, long j) {
        return get((this.strides[0] * i) + j);
    }

    public Bfloat16Indexer get(long i, long j, float[] h, int offset, int length) {
        for (int n = 0; n < length; n++) {
            h[offset + n] = get((this.strides[0] * i) + (this.strides[1] * j) + ((long) n));
        }
        return this;
    }

    public float get(long i, long j, long k) {
        return get((this.strides[0] * i) + (this.strides[1] * j) + k);
    }

    public float get(long... indices) {
        return get(index(indices));
    }

    public Bfloat16Indexer get(long[] indices, float[] h, int offset, int length) {
        for (int n = 0; n < length; n++) {
            h[offset + n] = get(index(indices) + ((long) n));
        }
        return this;
    }

    public Bfloat16Indexer put(long i, float h) {
        RAW.putShort(this.base + (checkIndex(i, this.size) * 2), (short) fromFloat(h));
        return this;
    }

    public Bfloat16Indexer put(long i, float[] h, int offset, int length) {
        for (int n = 0; n < length; n++) {
            put((this.strides[0] * i) + ((long) n), h[offset + n]);
        }
        return this;
    }

    public Bfloat16Indexer put(long i, long j, float h) {
        put((this.strides[0] * i) + j, h);
        return this;
    }

    public Bfloat16Indexer put(long i, long j, float[] h, int offset, int length) {
        for (int n = 0; n < length; n++) {
            put((this.strides[0] * i) + (this.strides[1] * j) + ((long) n), h[offset + n]);
        }
        return this;
    }

    public Bfloat16Indexer put(long i, long j, long k, float h) {
        put((this.strides[0] * i) + (this.strides[1] * j) + k, h);
        return this;
    }

    public Bfloat16Indexer put(long[] indices, float h) {
        put(index(indices), h);
        return this;
    }

    public Bfloat16Indexer put(long[] indices, float[] h, int offset, int length) {
        for (int n = 0; n < length; n++) {
            put(index(indices) + ((long) n), h[offset + n]);
        }
        return this;
    }

    public void release() {
        this.pointer = null;
    }
}
