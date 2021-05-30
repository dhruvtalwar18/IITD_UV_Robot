package org.bytedeco.javacpp.indexer;

import java.nio.Buffer;
import java.nio.ByteBuffer;

public class BooleanBufferIndexer extends BooleanIndexer {
    protected ByteBuffer buffer;

    public BooleanBufferIndexer(ByteBuffer buffer2) {
        this(buffer2, new long[]{(long) buffer2.limit()}, ONE_STRIDE);
    }

    public BooleanBufferIndexer(ByteBuffer buffer2, long[] sizes, long[] strides) {
        super(sizes, strides);
        this.buffer = buffer2;
    }

    public Buffer buffer() {
        return this.buffer;
    }

    public boolean get(long i) {
        return this.buffer.get((int) i) != 0;
    }

    public BooleanIndexer get(long i, boolean[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            b[offset + n] = this.buffer.get((((int) i) * ((int) this.strides[0])) + n) != 0;
        }
        return this;
    }

    public boolean get(long i, long j) {
        return this.buffer.get((((int) i) * ((int) this.strides[0])) + ((int) j)) != 0;
    }

    public BooleanIndexer get(long i, long j, boolean[] b, int offset, int length) {
        char c = 0;
        int n = 0;
        while (n < length) {
            int i2 = offset + n;
            boolean z = true;
            int n2 = n;
            if (this.buffer.get((((int) i) * ((int) this.strides[c])) + (((int) j) * ((int) this.strides[1])) + n2) == 0) {
                z = false;
            }
            b[i2] = z;
            n = n2 + 1;
            c = 0;
        }
        long j2 = i;
        long j3 = j;
        return this;
    }

    public boolean get(long i, long j, long k) {
        return this.buffer.get(((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1]))) + ((int) k)) != 0;
    }

    public boolean get(long... indices) {
        return this.buffer.get((int) index(indices)) != 0;
    }

    public BooleanIndexer get(long[] indices, boolean[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            b[offset + n] = this.buffer.get(((int) index(indices)) + n) != 0;
        }
        return this;
    }

    public BooleanIndexer put(long i, boolean b) {
        this.buffer.put((int) i, b);
        return this;
    }

    public BooleanIndexer put(long i, boolean[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put((((int) i) * ((int) this.strides[0])) + n, b[offset + n] ? (byte) 1 : 0);
        }
        return this;
    }

    public BooleanIndexer put(long i, long j, boolean b) {
        this.buffer.put((((int) i) * ((int) this.strides[0])) + ((int) j), b);
        return this;
    }

    public BooleanIndexer put(long i, long j, boolean[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n, b[offset + n] ? (byte) 1 : 0);
        }
        return this;
    }

    public BooleanIndexer put(long i, long j, long k, boolean b) {
        this.buffer.put((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k), b);
        return this;
    }

    public BooleanIndexer put(long[] indices, boolean b) {
        this.buffer.put((int) index(indices), b);
        return this;
    }

    public BooleanIndexer put(long[] indices, boolean[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put(((int) index(indices)) + n, b[offset + n] ? (byte) 1 : 0);
        }
        return this;
    }

    public void release() {
        this.buffer = null;
    }
}
