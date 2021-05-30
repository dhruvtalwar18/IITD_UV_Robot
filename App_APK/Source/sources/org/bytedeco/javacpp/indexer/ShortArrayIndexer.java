package org.bytedeco.javacpp.indexer;

public class ShortArrayIndexer extends ShortIndexer {
    protected short[] array;

    public /* bridge */ /* synthetic */ Indexer putDouble(long[] jArr, double d) {
        return super.putDouble(jArr, d);
    }

    public ShortArrayIndexer(short[] array2) {
        this(array2, new long[]{(long) array2.length}, ONE_STRIDE);
    }

    public ShortArrayIndexer(short[] array2, long[] sizes, long[] strides) {
        super(sizes, strides);
        this.array = array2;
    }

    public short[] array() {
        return this.array;
    }

    public short get(long i) {
        return this.array[(int) i];
    }

    public ShortIndexer get(long i, short[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            s[offset + n] = this.array[(((int) i) * ((int) this.strides[0])) + n];
        }
        return this;
    }

    public short get(long i, long j) {
        return this.array[(((int) i) * ((int) this.strides[0])) + ((int) j)];
    }

    public ShortIndexer get(long i, long j, short[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            s[offset + n] = this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n];
        }
        long j2 = i;
        long j3 = j;
        return this;
    }

    public short get(long i, long j, long k) {
        return this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k)];
    }

    public short get(long... indices) {
        return this.array[(int) index(indices)];
    }

    public ShortIndexer get(long[] indices, short[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            s[offset + n] = this.array[((int) index(indices)) + n];
        }
        return this;
    }

    public ShortIndexer put(long i, short s) {
        this.array[(int) i] = s;
        return this;
    }

    public ShortIndexer put(long i, short[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.array[(((int) i) * ((int) this.strides[0])) + n] = s[offset + n];
        }
        return this;
    }

    public ShortIndexer put(long i, long j, short s) {
        this.array[(((int) i) * ((int) this.strides[0])) + ((int) j)] = s;
        return this;
    }

    public ShortIndexer put(long i, long j, short[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n] = s[offset + n];
        }
        return this;
    }

    public ShortIndexer put(long i, long j, long k, short s) {
        this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k)] = s;
        return this;
    }

    public ShortIndexer put(long[] indices, short s) {
        this.array[(int) index(indices)] = s;
        return this;
    }

    public ShortIndexer put(long[] indices, short[] s, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.array[((int) index(indices)) + n] = s[offset + n];
        }
        return this;
    }

    public void release() {
        this.array = null;
    }
}
