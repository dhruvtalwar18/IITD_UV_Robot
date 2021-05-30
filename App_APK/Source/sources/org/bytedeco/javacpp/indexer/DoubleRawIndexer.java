package org.bytedeco.javacpp.indexer;

import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.javacpp.Pointer;

public class DoubleRawIndexer extends DoubleIndexer {
    protected static final Raw RAW = Raw.getInstance();
    final long base;
    protected DoublePointer pointer;
    final long size;

    public DoubleRawIndexer(DoublePointer pointer2) {
        this(pointer2, new long[]{pointer2.limit() - pointer2.position()}, ONE_STRIDE);
    }

    public DoubleRawIndexer(DoublePointer pointer2, long[] sizes, long[] strides) {
        super(sizes, strides);
        this.pointer = pointer2;
        this.base = pointer2.address() + (pointer2.position() * 8);
        this.size = pointer2.limit() - pointer2.position();
    }

    public Pointer pointer() {
        return this.pointer;
    }

    public double get(long i) {
        return RAW.getDouble(this.base + (checkIndex(i, this.size) * 8));
    }

    public DoubleIndexer get(long i, double[] d, int offset, int length) {
        for (int n = 0; n < length; n++) {
            d[offset + n] = get((this.strides[0] * i) + ((long) n));
        }
        return this;
    }

    public double get(long i, long j) {
        return get((this.strides[0] * i) + j);
    }

    public DoubleIndexer get(long i, long j, double[] d, int offset, int length) {
        for (int n = 0; n < length; n++) {
            d[offset + n] = get((this.strides[0] * i) + (this.strides[1] * j) + ((long) n));
        }
        return this;
    }

    public double get(long i, long j, long k) {
        return get((this.strides[0] * i) + (this.strides[1] * j) + k);
    }

    public double get(long... indices) {
        return get(index(indices));
    }

    public DoubleIndexer get(long[] indices, double[] d, int offset, int length) {
        for (int n = 0; n < length; n++) {
            d[offset + n] = get(index(indices) + ((long) n));
        }
        return this;
    }

    public DoubleIndexer put(long i, double d) {
        RAW.putDouble(this.base + (checkIndex(i, this.size) * 8), d);
        return this;
    }

    public DoubleIndexer put(long i, double[] d, int offset, int length) {
        for (int n = 0; n < length; n++) {
            put((this.strides[0] * i) + ((long) n), d[offset + n]);
        }
        return this;
    }

    public DoubleIndexer put(long i, long j, double d) {
        put((this.strides[0] * i) + j, d);
        return this;
    }

    public DoubleIndexer put(long i, long j, double[] d, int offset, int length) {
        for (int n = 0; n < length; n++) {
            put((this.strides[0] * i) + (this.strides[1] * j) + ((long) n), d[offset + n]);
        }
        return this;
    }

    public DoubleIndexer put(long i, long j, long k, double d) {
        put((this.strides[0] * i) + (this.strides[1] * j) + k, d);
        return this;
    }

    public DoubleIndexer put(long[] indices, double d) {
        put(index(indices), d);
        return this;
    }

    public DoubleIndexer put(long[] indices, double[] d, int offset, int length) {
        for (int n = 0; n < length; n++) {
            put(index(indices) + ((long) n), d[offset + n]);
        }
        return this;
    }

    public void release() {
        this.pointer = null;
    }
}
