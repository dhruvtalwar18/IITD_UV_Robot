package org.bytedeco.javacpp.indexer;

import org.bytedeco.javacpp.CharPointer;
import org.bytedeco.javacpp.Pointer;

public class CharRawIndexer extends CharIndexer {
    protected static final Raw RAW = Raw.getInstance();
    final long base;
    protected CharPointer pointer;
    final long size;

    public CharRawIndexer(CharPointer pointer2) {
        this(pointer2, new long[]{pointer2.limit() - pointer2.position()}, ONE_STRIDE);
    }

    public CharRawIndexer(CharPointer pointer2, long[] sizes, long[] strides) {
        super(sizes, strides);
        this.pointer = pointer2;
        this.base = pointer2.address() + (pointer2.position() * 2);
        this.size = pointer2.limit() - pointer2.position();
    }

    public Pointer pointer() {
        return this.pointer;
    }

    public char get(long i) {
        return RAW.getChar(this.base + (checkIndex(i, this.size) * 2));
    }

    public CharIndexer get(long i, char[] c, int offset, int length) {
        for (int n = 0; n < length; n++) {
            c[offset + n] = get((this.strides[0] * i) + ((long) n));
        }
        return this;
    }

    public char get(long i, long j) {
        return get((this.strides[0] * i) + j);
    }

    public CharIndexer get(long i, long j, char[] c, int offset, int length) {
        for (int n = 0; n < length; n++) {
            c[offset + n] = get((this.strides[0] * i) + (this.strides[1] * j) + ((long) n));
        }
        return this;
    }

    public char get(long i, long j, long k) {
        return get((this.strides[0] * i) + (this.strides[1] * j) + k);
    }

    public char get(long... indices) {
        return get(index(indices));
    }

    public CharIndexer get(long[] indices, char[] c, int offset, int length) {
        for (int n = 0; n < length; n++) {
            c[offset + n] = get(index(indices) + ((long) n));
        }
        return this;
    }

    public CharIndexer put(long i, char c) {
        RAW.putChar(this.base + (checkIndex(i, this.size) * 2), c);
        return this;
    }

    public CharIndexer put(long i, char[] c, int offset, int length) {
        for (int n = 0; n < length; n++) {
            put((this.strides[0] * i) + ((long) n), c[offset + n]);
        }
        return this;
    }

    public CharIndexer put(long i, long j, char c) {
        put((this.strides[0] * i) + j, c);
        return this;
    }

    public CharIndexer put(long i, long j, char[] c, int offset, int length) {
        for (int n = 0; n < length; n++) {
            put((this.strides[0] * i) + (this.strides[1] * j) + ((long) n), c[offset + n]);
        }
        return this;
    }

    public CharIndexer put(long i, long j, long k, char c) {
        put((this.strides[0] * i) + (this.strides[1] * j) + k, c);
        return this;
    }

    public CharIndexer put(long[] indices, char c) {
        put(index(indices), c);
        return this;
    }

    public CharIndexer put(long[] indices, char[] c, int offset, int length) {
        for (int n = 0; n < length; n++) {
            put(index(indices) + ((long) n), c[offset + n]);
        }
        return this;
    }

    public void release() {
        this.pointer = null;
    }
}
