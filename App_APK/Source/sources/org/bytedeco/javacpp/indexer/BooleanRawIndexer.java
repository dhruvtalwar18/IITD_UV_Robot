package org.bytedeco.javacpp.indexer;

import org.bytedeco.javacpp.BooleanPointer;
import org.bytedeco.javacpp.Pointer;

public class BooleanRawIndexer extends BooleanIndexer {
    protected static final Raw RAW = Raw.getInstance();
    final long base;
    protected BooleanPointer pointer;
    final long size;

    public BooleanRawIndexer(BooleanPointer pointer2) {
        this(pointer2, new long[]{pointer2.limit() - pointer2.position()}, ONE_STRIDE);
    }

    public BooleanRawIndexer(BooleanPointer pointer2, long[] sizes, long[] strides) {
        super(sizes, strides);
        this.pointer = pointer2;
        this.base = pointer2.address() + (pointer2.position() * 1);
        this.size = pointer2.limit() - pointer2.position();
    }

    public Pointer pointer() {
        return this.pointer;
    }

    public boolean get(long i) {
        return RAW.getBoolean(this.base + (checkIndex(i, this.size) * 1));
    }

    public BooleanIndexer get(long i, boolean[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            b[offset + n] = get((this.strides[0] * i) + ((long) n));
        }
        return this;
    }

    public boolean get(long i, long j) {
        return get((this.strides[0] * i) + j);
    }

    public BooleanIndexer get(long i, long j, boolean[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            b[offset + n] = get((this.strides[0] * i) + (this.strides[1] * j) + ((long) n));
        }
        return this;
    }

    public boolean get(long i, long j, long k) {
        return get((this.strides[0] * i) + (this.strides[1] * j) + k);
    }

    public boolean get(long... indices) {
        return get(index(indices));
    }

    public BooleanIndexer get(long[] indices, boolean[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            b[offset + n] = get(index(indices) + ((long) n));
        }
        return this;
    }

    public BooleanIndexer put(long i, boolean b) {
        RAW.putBoolean(this.base + (checkIndex(i, this.size) * 1), b);
        return this;
    }

    public BooleanIndexer put(long i, boolean[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            put((this.strides[0] * i) + ((long) n), b[offset + n]);
        }
        return this;
    }

    public BooleanIndexer put(long i, long j, boolean b) {
        put((this.strides[0] * i) + j, b);
        return this;
    }

    public BooleanIndexer put(long i, long j, boolean[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            put((this.strides[0] * i) + (this.strides[1] * j) + ((long) n), b[offset + n]);
        }
        return this;
    }

    public BooleanIndexer put(long i, long j, long k, boolean b) {
        put((this.strides[0] * i) + (this.strides[1] * j) + k, b);
        return this;
    }

    public BooleanIndexer put(long[] indices, boolean b) {
        put(index(indices), b);
        return this;
    }

    public BooleanIndexer put(long[] indices, boolean[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            put(index(indices) + ((long) n), b[offset + n]);
        }
        return this;
    }

    public void release() {
        this.pointer = null;
    }
}
