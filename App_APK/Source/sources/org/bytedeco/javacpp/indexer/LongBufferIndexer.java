package org.bytedeco.javacpp.indexer;

import java.nio.Buffer;
import java.nio.LongBuffer;

public class LongBufferIndexer extends LongIndexer {
    protected LongBuffer buffer;

    public LongBufferIndexer(LongBuffer buffer2) {
        this(buffer2, new long[]{(long) buffer2.limit()}, ONE_STRIDE);
    }

    public LongBufferIndexer(LongBuffer buffer2, long[] sizes, long[] strides) {
        super(sizes, strides);
        this.buffer = buffer2;
    }

    public Buffer buffer() {
        return this.buffer;
    }

    public long get(long i) {
        return this.buffer.get((int) i);
    }

    public LongIndexer get(long i, long[] l, int offset, int length) {
        for (int n = 0; n < length; n++) {
            l[offset + n] = this.buffer.get((((int) i) * ((int) this.strides[0])) + n);
        }
        return this;
    }

    public long get(long i, long j) {
        return this.buffer.get((((int) i) * ((int) this.strides[0])) + ((int) j));
    }

    public LongIndexer get(long i, long j, long[] l, int offset, int length) {
        for (int n = 0; n < length; n++) {
            l[offset + n] = this.buffer.get((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n);
        }
        long j2 = i;
        long j3 = j;
        return this;
    }

    public long get(long i, long j, long k) {
        return this.buffer.get((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k));
    }

    public long get(long... indices) {
        return this.buffer.get((int) index(indices));
    }

    public LongIndexer get(long[] indices, long[] l, int offset, int length) {
        for (int n = 0; n < length; n++) {
            l[offset + n] = this.buffer.get(((int) index(indices)) + n);
        }
        return this;
    }

    public LongIndexer put(long i, long l) {
        this.buffer.put((int) i, l);
        return this;
    }

    public LongIndexer put(long i, long[] l, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put((((int) i) * ((int) this.strides[0])) + n, l[offset + n]);
        }
        return this;
    }

    public LongIndexer put(long i, long j, long l) {
        this.buffer.put((((int) i) * ((int) this.strides[0])) + ((int) j), l);
        return this;
    }

    public LongIndexer put(long i, long j, long[] l, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n, l[offset + n]);
        }
        return this;
    }

    public LongIndexer put(long i, long j, long k, long l) {
        this.buffer.put((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k), l);
        return this;
    }

    public LongIndexer put(long[] indices, long l) {
        this.buffer.put((int) index(indices), l);
        return this;
    }

    public LongIndexer put(long[] indices, long[] l, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put(((int) index(indices)) + n, l[offset + n]);
        }
        return this;
    }

    public void release() {
        this.buffer = null;
    }
}
