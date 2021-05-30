package org.bytedeco.javacpp.indexer;

public class UShortArrayIndexer extends UShortIndexer {
    protected short[] array;

    public /* bridge */ /* synthetic */ Indexer putDouble(long[] jArr, double d) {
        return super.putDouble(jArr, d);
    }

    public UShortArrayIndexer(short[] array2) {
        this(array2, new long[]{(long) array2.length}, ONE_STRIDE);
    }

    public UShortArrayIndexer(short[] array2, long[] sizes, long[] strides) {
        super(sizes, strides);
        this.array = array2;
    }

    public short[] array() {
        return this.array;
    }

    public int get(long i) {
        return this.array[(int) i] & 65535;
    }

    public UShortIndexer get(long i, int[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            s[offset + n] = this.array[(((int) i) * ((int) this.strides[0])) + n] & 65535;
        }
        return this;
    }

    public int get(long i, long j) {
        return this.array[(((int) i) * ((int) this.strides[0])) + ((int) j)] & 65535;
    }

    public UShortIndexer get(long i, long j, int[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            s[offset + n] = this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n] & 65535;
        }
        long j2 = i;
        long j3 = j;
        return this;
    }

    public int get(long i, long j, long k) {
        return this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k)] & 65535;
    }

    public int get(long... indices) {
        return this.array[(int) index(indices)] & 65535;
    }

    public UShortIndexer get(long[] indices, int[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            s[offset + n] = this.array[((int) index(indices)) + n] & 65535;
        }
        return this;
    }

    public UShortIndexer put(long i, int s) {
        this.array[(int) i] = (short) s;
        return this;
    }

    public UShortIndexer put(long i, int[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.array[(((int) i) * ((int) this.strides[0])) + n] = (short) s[offset + n];
        }
        return this;
    }

    public UShortIndexer put(long i, long j, int s) {
        this.array[(((int) i) * ((int) this.strides[0])) + ((int) j)] = (short) s;
        return this;
    }

    public UShortIndexer put(long i, long j, int[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n] = (short) s[offset + n];
        }
        return this;
    }

    public UShortIndexer put(long i, long j, long k, int s) {
        this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k)] = (short) s;
        return this;
    }

    public UShortIndexer put(long[] indices, int s) {
        this.array[(int) index(indices)] = (short) s;
        return this;
    }

    public UShortIndexer put(long[] indices, int[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.array[((int) index(indices)) + n] = (short) s[offset + n];
        }
        return this;
    }

    public void release() {
        this.array = null;
    }
}
