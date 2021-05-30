package org.bytedeco.javacpp.indexer;

import java.nio.Buffer;
import java.nio.ShortBuffer;

public class UShortBufferIndexer extends UShortIndexer {
    protected ShortBuffer buffer;

    public UShortBufferIndexer(ShortBuffer buffer2) {
        this(buffer2, new long[]{(long) buffer2.limit()}, ONE_STRIDE);
    }

    public UShortBufferIndexer(ShortBuffer buffer2, long[] sizes, long[] strides) {
        super(sizes, strides);
        this.buffer = buffer2;
    }

    public Buffer buffer() {
        return this.buffer;
    }

    public int get(long i) {
        return this.buffer.get((int) i) & 65535;
    }

    public UShortIndexer get(long i, int[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            s[offset + n] = this.buffer.get((((int) i) * ((int) this.strides[0])) + n) & 65535;
        }
        return this;
    }

    public int get(long i, long j) {
        return this.buffer.get((((int) i) * ((int) this.strides[0])) + ((int) j)) & 65535;
    }

    public UShortIndexer get(long i, long j, int[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            s[offset + n] = this.buffer.get((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n) & 65535;
        }
        long j2 = i;
        long j3 = j;
        return this;
    }

    public int get(long i, long j, long k) {
        return this.buffer.get((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k)) & 65535;
    }

    public int get(long... indices) {
        return this.buffer.get((int) index(indices)) & 65535;
    }

    public UShortIndexer get(long[] indices, int[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            s[offset + n] = this.buffer.get(((int) index(indices)) + n) & 65535;
        }
        return this;
    }

    public UShortIndexer put(long i, int s) {
        this.buffer.put((int) i, (short) s);
        return this;
    }

    public UShortIndexer put(long i, int[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put((((int) i) * ((int) this.strides[0])) + n, (short) s[offset + n]);
        }
        return this;
    }

    public UShortIndexer put(long i, long j, int s) {
        this.buffer.put((((int) i) * ((int) this.strides[0])) + ((int) j), (short) s);
        return this;
    }

    public UShortIndexer put(long i, long j, int[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n, (short) s[offset + n]);
        }
        return this;
    }

    public UShortIndexer put(long i, long j, long k, int s) {
        this.buffer.put((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k), (short) s);
        return this;
    }

    public UShortIndexer put(long[] indices, int s) {
        this.buffer.put((int) index(indices), (short) s);
        return this;
    }

    public UShortIndexer put(long[] indices, int[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put(((int) index(indices)) + n, (short) s[offset + n]);
        }
        return this;
    }

    public void release() {
        this.buffer = null;
    }
}
