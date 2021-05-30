package org.bytedeco.javacpp.indexer;

import java.nio.Buffer;
import java.nio.IntBuffer;

public class IntBufferIndexer extends IntIndexer {
    protected IntBuffer buffer;

    public IntBufferIndexer(IntBuffer buffer2) {
        this(buffer2, new long[]{(long) buffer2.limit()}, ONE_STRIDE);
    }

    public IntBufferIndexer(IntBuffer buffer2, long[] sizes, long[] strides) {
        super(sizes, strides);
        this.buffer = buffer2;
    }

    public Buffer buffer() {
        return this.buffer;
    }

    public int get(long i) {
        return this.buffer.get((int) i);
    }

    public IntIndexer get(long i, int[] m, int offset, int length) {
        for (int n = 0; n < length; n++) {
            m[offset + n] = this.buffer.get((((int) i) * ((int) this.strides[0])) + n);
        }
        return this;
    }

    public int get(long i, long j) {
        return this.buffer.get((((int) i) * ((int) this.strides[0])) + ((int) j));
    }

    public IntIndexer get(long i, long j, int[] m, int offset, int length) {
        for (int n = 0; n < length; n++) {
            m[offset + n] = this.buffer.get((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n);
        }
        long j2 = i;
        long j3 = j;
        return this;
    }

    public int get(long i, long j, long k) {
        return this.buffer.get((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k));
    }

    public int get(long... indices) {
        return this.buffer.get((int) index(indices));
    }

    public IntIndexer get(long[] indices, int[] m, int offset, int length) {
        for (int n = 0; n < length; n++) {
            m[offset + n] = this.buffer.get(((int) index(indices)) + n);
        }
        return this;
    }

    public IntIndexer put(long i, int n) {
        this.buffer.put((int) i, n);
        return this;
    }

    public IntIndexer put(long i, int[] m, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put((((int) i) * ((int) this.strides[0])) + n, m[offset + n]);
        }
        return this;
    }

    public IntIndexer put(long i, long j, int n) {
        this.buffer.put((((int) i) * ((int) this.strides[0])) + ((int) j), n);
        return this;
    }

    public IntIndexer put(long i, long j, int[] m, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n, m[offset + n]);
        }
        return this;
    }

    public IntIndexer put(long i, long j, long k, int n) {
        this.buffer.put((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k), n);
        return this;
    }

    public IntIndexer put(long[] indices, int n) {
        this.buffer.put((int) index(indices), n);
        return this;
    }

    public IntIndexer put(long[] indices, int[] m, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put(((int) index(indices)) + n, m[offset + n]);
        }
        return this;
    }

    public void release() {
        this.buffer = null;
    }
}
