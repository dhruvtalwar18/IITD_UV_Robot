package org.bytedeco.javacpp.indexer;

import java.nio.Buffer;
import java.nio.ShortBuffer;

public class ShortBufferIndexer extends ShortIndexer {
    protected ShortBuffer buffer;

    public ShortBufferIndexer(ShortBuffer buffer2) {
        this(buffer2, new long[]{(long) buffer2.limit()}, ONE_STRIDE);
    }

    public ShortBufferIndexer(ShortBuffer buffer2, long[] sizes, long[] strides) {
        super(sizes, strides);
        this.buffer = buffer2;
    }

    public Buffer buffer() {
        return this.buffer;
    }

    public short get(long i) {
        return this.buffer.get((int) i);
    }

    public ShortIndexer get(long i, short[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            s[offset + n] = this.buffer.get((((int) i) * ((int) this.strides[0])) + n);
        }
        return this;
    }

    public short get(long i, long j) {
        return this.buffer.get((((int) i) * ((int) this.strides[0])) + ((int) j));
    }

    public ShortIndexer get(long i, long j, short[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            s[offset + n] = this.buffer.get((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n);
        }
        long j2 = i;
        long j3 = j;
        return this;
    }

    public short get(long i, long j, long k) {
        return this.buffer.get((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k));
    }

    public short get(long... indices) {
        return this.buffer.get((int) index(indices));
    }

    public ShortIndexer get(long[] indices, short[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            s[offset + n] = this.buffer.get(((int) index(indices)) + n);
        }
        return this;
    }

    public ShortIndexer put(long i, short s) {
        this.buffer.put((int) i, s);
        return this;
    }

    public ShortIndexer put(long i, short[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put((((int) i) * ((int) this.strides[0])) + n, s[offset + n]);
        }
        return this;
    }

    public ShortIndexer put(long i, long j, short s) {
        this.buffer.put((((int) i) * ((int) this.strides[0])) + ((int) j), s);
        return this;
    }

    public ShortIndexer put(long i, long j, short[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n, s[offset + n]);
        }
        return this;
    }

    public ShortIndexer put(long i, long j, long k, short s) {
        this.buffer.put((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k), s);
        return this;
    }

    public ShortIndexer put(long[] indices, short s) {
        this.buffer.put((int) index(indices), s);
        return this;
    }

    public ShortIndexer put(long[] indices, short[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put(((int) index(indices)) + n, s[offset + n]);
        }
        return this;
    }

    public void release() {
        this.buffer = null;
    }
}
