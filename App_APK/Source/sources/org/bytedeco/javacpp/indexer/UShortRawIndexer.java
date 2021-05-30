package org.bytedeco.javacpp.indexer;

import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacpp.ShortPointer;

public class UShortRawIndexer extends UShortIndexer {
    protected static final Raw RAW = Raw.getInstance();
    final long base;
    protected ShortPointer pointer;
    final long size;

    public UShortRawIndexer(ShortPointer pointer2) {
        this(pointer2, new long[]{pointer2.limit() - pointer2.position()}, ONE_STRIDE);
    }

    public UShortRawIndexer(ShortPointer pointer2, long[] sizes, long[] strides) {
        super(sizes, strides);
        this.pointer = pointer2;
        this.base = pointer2.address() + (pointer2.position() * 2);
        this.size = pointer2.limit() - pointer2.position();
    }

    public Pointer pointer() {
        return this.pointer;
    }

    public int get(long i) {
        return RAW.getShort(this.base + (checkIndex(i, this.size) * 2)) & 65535;
    }

    public UShortIndexer get(long i, int[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            s[offset + n] = get((this.strides[0] * i) + ((long) n)) & 65535;
        }
        return this;
    }

    public int get(long i, long j) {
        return get((this.strides[0] * i) + j) & 65535;
    }

    public UShortIndexer get(long i, long j, int[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            s[offset + n] = get((this.strides[0] * i) + (this.strides[1] * j) + ((long) n)) & 65535;
        }
        return this;
    }

    public int get(long i, long j, long k) {
        return get((this.strides[0] * i) + (this.strides[1] * j) + k) & 65535;
    }

    public int get(long... indices) {
        return get(index(indices)) & 65535;
    }

    public UShortIndexer get(long[] indices, int[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            s[offset + n] = get(index(indices) + ((long) n)) & 65535;
        }
        return this;
    }

    public UShortIndexer put(long i, int s) {
        RAW.putShort(this.base + (checkIndex(i, this.size) * 2), (short) s);
        return this;
    }

    public UShortIndexer put(long i, int[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            put((this.strides[0] * i) + ((long) n), (int) (short) s[offset + n]);
        }
        return this;
    }

    public UShortIndexer put(long i, long j, int s) {
        put((this.strides[0] * i) + j, (int) (short) s);
        return this;
    }

    public UShortIndexer put(long i, long j, int[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            put((this.strides[0] * i) + (this.strides[1] * j) + ((long) n), (int) (short) s[offset + n]);
        }
        return this;
    }

    public UShortIndexer put(long i, long j, long k, int s) {
        put((this.strides[0] * i) + (this.strides[1] * j) + k, (int) (short) s);
        return this;
    }

    public UShortIndexer put(long[] indices, int s) {
        put(index(indices), (int) (short) s);
        return this;
    }

    public UShortIndexer put(long[] indices, int[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            put(index(indices) + ((long) n), (int) (short) s[offset + n]);
        }
        return this;
    }

    public void release() {
        this.pointer = null;
    }
}
