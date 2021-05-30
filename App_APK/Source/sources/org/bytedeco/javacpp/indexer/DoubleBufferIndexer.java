package org.bytedeco.javacpp.indexer;

import java.nio.Buffer;
import java.nio.DoubleBuffer;

public class DoubleBufferIndexer extends DoubleIndexer {
    protected DoubleBuffer buffer;

    public DoubleBufferIndexer(DoubleBuffer buffer2) {
        this(buffer2, new long[]{(long) buffer2.limit()}, ONE_STRIDE);
    }

    public DoubleBufferIndexer(DoubleBuffer buffer2, long[] sizes, long[] strides) {
        super(sizes, strides);
        this.buffer = buffer2;
    }

    public Buffer buffer() {
        return this.buffer;
    }

    public double get(long i) {
        return this.buffer.get((int) i);
    }

    public DoubleIndexer get(long i, double[] d, int offset, int length) {
        for (int n = 0; n < length; n++) {
            d[offset + n] = this.buffer.get((((int) i) * ((int) this.strides[0])) + n);
        }
        return this;
    }

    public double get(long i, long j) {
        return this.buffer.get((((int) i) * ((int) this.strides[0])) + ((int) j));
    }

    public DoubleIndexer get(long i, long j, double[] d, int offset, int length) {
        for (int n = 0; n < length; n++) {
            d[offset + n] = this.buffer.get((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n);
        }
        long j2 = i;
        long j3 = j;
        return this;
    }

    public double get(long i, long j, long k) {
        return this.buffer.get((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k));
    }

    public double get(long... indices) {
        return this.buffer.get((int) index(indices));
    }

    public DoubleIndexer get(long[] indices, double[] d, int offset, int length) {
        for (int n = 0; n < length; n++) {
            d[offset + n] = this.buffer.get(((int) index(indices)) + n);
        }
        return this;
    }

    public DoubleIndexer put(long i, double d) {
        this.buffer.put((int) i, d);
        return this;
    }

    public DoubleIndexer put(long i, double[] d, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put((((int) i) * ((int) this.strides[0])) + n, d[offset + n]);
        }
        return this;
    }

    public DoubleIndexer put(long i, long j, double d) {
        this.buffer.put((((int) i) * ((int) this.strides[0])) + ((int) j), d);
        return this;
    }

    public DoubleIndexer put(long i, long j, double[] d, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n, d[offset + n]);
        }
        return this;
    }

    public DoubleIndexer put(long i, long j, long k, double d) {
        this.buffer.put((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k), d);
        return this;
    }

    public DoubleIndexer put(long[] indices, double d) {
        this.buffer.put((int) index(indices), d);
        return this;
    }

    public DoubleIndexer put(long[] indices, double[] d, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put(((int) index(indices)) + n, d[offset + n]);
        }
        return this;
    }

    public void release() {
        this.buffer = null;
    }
}
