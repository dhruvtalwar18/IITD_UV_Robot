package org.bytedeco.javacpp.indexer;

import org.bytedeco.javacpp.LongPointer;
import org.bytedeco.javacpp.Pointer;

public class LongRawIndexer extends LongIndexer {
    protected static final Raw RAW = Raw.getInstance();
    final long base;
    protected LongPointer pointer;
    final long size;

    public LongRawIndexer(LongPointer pointer2) {
        this(pointer2, new long[]{pointer2.limit() - pointer2.position()}, ONE_STRIDE);
    }

    public LongRawIndexer(LongPointer pointer2, long[] sizes, long[] strides) {
        super(sizes, strides);
        this.pointer = pointer2;
        this.base = pointer2.address() + (pointer2.position() * 8);
        this.size = pointer2.limit() - pointer2.position();
    }

    public Pointer pointer() {
        return this.pointer;
    }

    public long get(long i) {
        return RAW.getLong(this.base + (checkIndex(i, this.size) * 8));
    }

    public LongIndexer get(long i, long[] l, int offset, int length) {
        for (int n = 0; n < length; n++) {
            l[offset + n] = get((this.strides[0] * i) + ((long) n));
        }
        return this;
    }

    public long get(long i, long j) {
        return get((this.strides[0] * i) + j);
    }

    public LongIndexer get(long i, long j, long[] l, int offset, int length) {
        for (int n = 0; n < length; n++) {
            l[offset + n] = get((this.strides[0] * i) + (this.strides[1] * j) + ((long) n));
        }
        return this;
    }

    public long get(long i, long j, long k) {
        return get((this.strides[0] * i) + (this.strides[1] * j) + k);
    }

    public long get(long... indices) {
        return get(index(indices));
    }

    public LongIndexer get(long[] indices, long[] l, int offset, int length) {
        for (int n = 0; n < length; n++) {
            l[offset + n] = get(index(indices) + ((long) n));
        }
        return this;
    }

    public LongIndexer put(long i, long l) {
        RAW.putLong(this.base + (checkIndex(i, this.size) * 8), l);
        return this;
    }

    public LongIndexer put(long i, long[] l, int offset, int length) {
        for (int n = 0; n < length; n++) {
            put((this.strides[0] * i) + ((long) n), l[offset + n]);
        }
        return this;
    }

    public LongIndexer put(long i, long j, long l) {
        put((this.strides[0] * i) + j, l);
        return this;
    }

    public LongIndexer put(long i, long j, long[] l, int offset, int length) {
        for (int n = 0; n < length; n++) {
            put((this.strides[0] * i) + (this.strides[1] * j) + ((long) n), l[offset + n]);
        }
        return this;
    }

    public LongIndexer put(long i, long j, long k, long l) {
        put((this.strides[0] * i) + (this.strides[1] * j) + k, l);
        return this;
    }

    public LongIndexer put(long[] indices, long l) {
        put(index(indices), l);
        return this;
    }

    public LongIndexer put(long[] indices, long[] l, int offset, int length) {
        for (int n = 0; n < length; n++) {
            put(index(indices) + ((long) n), l[offset + n]);
        }
        return this;
    }

    public void release() {
        this.pointer = null;
    }
}
