package org.bytedeco.javacpp.indexer;

import java.nio.Buffer;
import java.nio.FloatBuffer;

public class FloatBufferIndexer extends FloatIndexer {
    protected FloatBuffer buffer;

    public FloatBufferIndexer(FloatBuffer buffer2) {
        this(buffer2, new long[]{(long) buffer2.limit()}, ONE_STRIDE);
    }

    public FloatBufferIndexer(FloatBuffer buffer2, long[] sizes, long[] strides) {
        super(sizes, strides);
        this.buffer = buffer2;
    }

    public Buffer buffer() {
        return this.buffer;
    }

    public float get(long i) {
        return this.buffer.get((int) i);
    }

    public FloatIndexer get(long i, float[] f, int offset, int length) {
        for (int n = 0; n < length; n++) {
            f[offset + n] = this.buffer.get((((int) i) * ((int) this.strides[0])) + n);
        }
        return this;
    }

    public float get(long i, long j) {
        return this.buffer.get((((int) i) * ((int) this.strides[0])) + ((int) j));
    }

    public FloatIndexer get(long i, long j, float[] f, int offset, int length) {
        for (int n = 0; n < length; n++) {
            f[offset + n] = this.buffer.get((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n);
        }
        long j2 = i;
        long j3 = j;
        return this;
    }

    public float get(long i, long j, long k) {
        return this.buffer.get((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k));
    }

    public float get(long... indices) {
        return this.buffer.get((int) index(indices));
    }

    public FloatIndexer get(long[] indices, float[] f, int offset, int length) {
        for (int n = 0; n < length; n++) {
            f[offset + n] = this.buffer.get(((int) index(indices)) + n);
        }
        return this;
    }

    public FloatIndexer put(long i, float f) {
        this.buffer.put((int) i, f);
        return this;
    }

    public FloatIndexer put(long i, float[] f, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put((((int) i) * ((int) this.strides[0])) + n, f[offset + n]);
        }
        return this;
    }

    public FloatIndexer put(long i, long j, float f) {
        this.buffer.put((((int) i) * ((int) this.strides[0])) + ((int) j), f);
        return this;
    }

    public FloatIndexer put(long i, long j, float[] f, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n, f[offset + n]);
        }
        return this;
    }

    public FloatIndexer put(long i, long j, long k, float f) {
        this.buffer.put((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k), f);
        return this;
    }

    public FloatIndexer put(long[] indices, float f) {
        this.buffer.put((int) index(indices), f);
        return this;
    }

    public FloatIndexer put(long[] indices, float[] f, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put(((int) index(indices)) + n, f[offset + n]);
        }
        return this;
    }

    public void release() {
        this.buffer = null;
    }
}
