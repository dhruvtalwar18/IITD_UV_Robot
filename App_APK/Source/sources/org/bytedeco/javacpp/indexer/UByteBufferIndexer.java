package org.bytedeco.javacpp.indexer;

import java.nio.Buffer;
import java.nio.ByteBuffer;
import sensor_msgs.NavSatStatus;

public class UByteBufferIndexer extends UByteIndexer {
    protected ByteBuffer buffer;

    public UByteBufferIndexer(ByteBuffer buffer2) {
        this(buffer2, new long[]{(long) buffer2.limit()}, ONE_STRIDE);
    }

    public UByteBufferIndexer(ByteBuffer buffer2, long[] sizes, long[] strides) {
        super(sizes, strides);
        this.buffer = buffer2;
    }

    public Buffer buffer() {
        return this.buffer;
    }

    public int get(long i) {
        return this.buffer.get((int) i) & NavSatStatus.STATUS_NO_FIX;
    }

    public UByteIndexer get(long i, int[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            b[offset + n] = this.buffer.get((((int) i) * ((int) this.strides[0])) + n) & NavSatStatus.STATUS_NO_FIX;
        }
        return this;
    }

    public int get(long i, long j) {
        return this.buffer.get((((int) i) * ((int) this.strides[0])) + ((int) j)) & NavSatStatus.STATUS_NO_FIX;
    }

    public UByteIndexer get(long i, long j, int[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            b[offset + n] = this.buffer.get((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n) & NavSatStatus.STATUS_NO_FIX;
        }
        long j2 = i;
        long j3 = j;
        return this;
    }

    public int get(long i, long j, long k) {
        return this.buffer.get((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k)) & NavSatStatus.STATUS_NO_FIX;
    }

    public int get(long... indices) {
        return this.buffer.get((int) index(indices)) & NavSatStatus.STATUS_NO_FIX;
    }

    public UByteIndexer get(long[] indices, int[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            b[offset + n] = this.buffer.get(((int) index(indices)) + n) & NavSatStatus.STATUS_NO_FIX;
        }
        return this;
    }

    public UByteIndexer put(long i, int b) {
        this.buffer.put((int) i, (byte) b);
        return this;
    }

    public UByteIndexer put(long i, int[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put((((int) i) * ((int) this.strides[0])) + n, (byte) b[offset + n]);
        }
        return this;
    }

    public UByteIndexer put(long i, long j, int b) {
        this.buffer.put((((int) i) * ((int) this.strides[0])) + ((int) j), (byte) b);
        return this;
    }

    public UByteIndexer put(long i, long j, int[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n, (byte) b[offset + n]);
        }
        return this;
    }

    public UByteIndexer put(long i, long j, long k, int b) {
        this.buffer.put((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k), (byte) b);
        return this;
    }

    public UByteIndexer put(long[] indices, int b) {
        this.buffer.put((int) index(indices), (byte) b);
        return this;
    }

    public UByteIndexer put(long[] indices, int[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put(((int) index(indices)) + n, (byte) b[offset + n]);
        }
        return this;
    }

    public void release() {
        this.buffer = null;
    }
}
