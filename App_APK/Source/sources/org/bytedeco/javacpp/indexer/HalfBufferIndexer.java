package org.bytedeco.javacpp.indexer;

import java.nio.Buffer;
import java.nio.ShortBuffer;

public class HalfBufferIndexer extends HalfIndexer {
    protected ShortBuffer buffer;

    public HalfBufferIndexer(ShortBuffer buffer2) {
        this(buffer2, new long[]{(long) buffer2.limit()}, ONE_STRIDE);
    }

    public HalfBufferIndexer(ShortBuffer buffer2, long[] sizes, long[] strides) {
        super(sizes, strides);
        this.buffer = buffer2;
    }

    public Buffer buffer() {
        return this.buffer;
    }

    public float get(long i) {
        return toFloat(this.buffer.get((int) i));
    }

    public HalfIndexer get(long i, float[] h, int offset, int length) {
        for (int n = 0; n < length; n++) {
            h[offset + n] = toFloat(this.buffer.get((((int) i) * ((int) this.strides[0])) + n));
        }
        return this;
    }

    public float get(long i, long j) {
        return toFloat(this.buffer.get((((int) i) * ((int) this.strides[0])) + ((int) j)));
    }

    public HalfIndexer get(long i, long j, float[] h, int offset, int length) {
        for (int n = 0; n < length; n++) {
            h[offset + n] = toFloat(this.buffer.get((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n));
        }
        long j2 = i;
        long j3 = j;
        return this;
    }

    public float get(long i, long j, long k) {
        return toFloat(this.buffer.get((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k)));
    }

    public float get(long... indices) {
        return toFloat(this.buffer.get((int) index(indices)));
    }

    public HalfIndexer get(long[] indices, float[] h, int offset, int length) {
        for (int n = 0; n < length; n++) {
            h[offset + n] = toFloat(this.buffer.get(((int) index(indices)) + n));
        }
        return this;
    }

    public HalfIndexer put(long i, float h) {
        this.buffer.put((int) i, (short) fromFloat(h));
        return this;
    }

    public HalfIndexer put(long i, float[] h, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put((((int) i) * ((int) this.strides[0])) + n, (short) fromFloat(h[offset + n]));
        }
        return this;
    }

    public HalfIndexer put(long i, long j, float h) {
        this.buffer.put((((int) i) * ((int) this.strides[0])) + ((int) j), (short) fromFloat(h));
        return this;
    }

    public HalfIndexer put(long i, long j, float[] h, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n, (short) fromFloat(h[offset + n]));
        }
        return this;
    }

    public HalfIndexer put(long i, long j, long k, float h) {
        this.buffer.put((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k), (short) fromFloat(h));
        return this;
    }

    public HalfIndexer put(long[] indices, float h) {
        this.buffer.put((int) index(indices), (short) fromFloat(h));
        return this;
    }

    public HalfIndexer put(long[] indices, float[] h, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put(((int) index(indices)) + n, (short) fromFloat(h[offset + n]));
        }
        return this;
    }

    public void release() {
        this.buffer = null;
    }
}
