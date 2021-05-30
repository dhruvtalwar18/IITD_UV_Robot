package org.bytedeco.javacpp.indexer;

import sensor_msgs.NavSatStatus;

public class UByteArrayIndexer extends UByteIndexer {
    protected byte[] array;

    public /* bridge */ /* synthetic */ Indexer putDouble(long[] jArr, double d) {
        return super.putDouble(jArr, d);
    }

    public UByteArrayIndexer(byte[] array2) {
        this(array2, new long[]{(long) array2.length}, ONE_STRIDE);
    }

    public UByteArrayIndexer(byte[] array2, long[] sizes, long[] strides) {
        super(sizes, strides);
        this.array = array2;
    }

    public byte[] array() {
        return this.array;
    }

    public int get(long i) {
        return this.array[(int) i] & NavSatStatus.STATUS_NO_FIX;
    }

    public UByteIndexer get(long i, int[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            b[offset + n] = this.array[(((int) i) * ((int) this.strides[0])) + n] & NavSatStatus.STATUS_NO_FIX;
        }
        return this;
    }

    public int get(long i, long j) {
        return this.array[(((int) i) * ((int) this.strides[0])) + ((int) j)] & NavSatStatus.STATUS_NO_FIX;
    }

    public UByteIndexer get(long i, long j, int[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            b[offset + n] = this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n] & NavSatStatus.STATUS_NO_FIX;
        }
        long j2 = i;
        long j3 = j;
        return this;
    }

    public int get(long i, long j, long k) {
        return this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k)] & NavSatStatus.STATUS_NO_FIX;
    }

    public int get(long... indices) {
        return this.array[(int) index(indices)] & NavSatStatus.STATUS_NO_FIX;
    }

    public UByteIndexer get(long[] indices, int[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            b[offset + n] = this.array[((int) index(indices)) + n] & NavSatStatus.STATUS_NO_FIX;
        }
        return this;
    }

    public UByteIndexer put(long i, int b) {
        this.array[(int) i] = (byte) b;
        return this;
    }

    public UByteIndexer put(long i, int[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.array[(((int) i) * ((int) this.strides[0])) + n] = (byte) b[offset + n];
        }
        return this;
    }

    public UByteIndexer put(long i, long j, int b) {
        this.array[(((int) i) * ((int) this.strides[0])) + ((int) j)] = (byte) b;
        return this;
    }

    public UByteIndexer put(long i, long j, int[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n] = (byte) b[offset + n];
        }
        return this;
    }

    public UByteIndexer put(long i, long j, long k, int b) {
        this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k)] = (byte) b;
        return this;
    }

    public UByteIndexer put(long[] indices, int b) {
        this.array[(int) index(indices)] = (byte) b;
        return this;
    }

    public UByteIndexer put(long[] indices, int[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.array[((int) index(indices)) + n] = (byte) b[offset + n];
        }
        return this;
    }

    public void release() {
        this.array = null;
    }
}
