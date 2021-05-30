package org.bytedeco.javacpp.indexer;

public class FloatArrayIndexer extends FloatIndexer {
    protected float[] array;

    public /* bridge */ /* synthetic */ Indexer putDouble(long[] jArr, double d) {
        return super.putDouble(jArr, d);
    }

    public FloatArrayIndexer(float[] array2) {
        this(array2, new long[]{(long) array2.length}, ONE_STRIDE);
    }

    public FloatArrayIndexer(float[] array2, long[] sizes, long[] strides) {
        super(sizes, strides);
        this.array = array2;
    }

    public float[] array() {
        return this.array;
    }

    public float get(long i) {
        return this.array[(int) i];
    }

    public FloatIndexer get(long i, float[] f, int offset, int length) {
        for (int n = 0; n < length; n++) {
            f[offset + n] = this.array[(((int) i) * ((int) this.strides[0])) + n];
        }
        return this;
    }

    public float get(long i, long j) {
        return this.array[(((int) i) * ((int) this.strides[0])) + ((int) j)];
    }

    public FloatIndexer get(long i, long j, float[] f, int offset, int length) {
        for (int n = 0; n < length; n++) {
            f[offset + n] = this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n];
        }
        long j2 = i;
        long j3 = j;
        return this;
    }

    public float get(long i, long j, long k) {
        return this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k)];
    }

    public float get(long... indices) {
        return this.array[(int) index(indices)];
    }

    public FloatIndexer get(long[] indices, float[] f, int offset, int length) {
        for (int n = 0; n < length; n++) {
            f[offset + n] = this.array[((int) index(indices)) + n];
        }
        return this;
    }

    public FloatIndexer put(long i, float f) {
        this.array[(int) i] = f;
        return this;
    }

    public FloatIndexer put(long i, float[] f, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.array[(((int) i) * ((int) this.strides[0])) + n] = f[offset + n];
        }
        return this;
    }

    public FloatIndexer put(long i, long j, float f) {
        this.array[(((int) i) * ((int) this.strides[0])) + ((int) j)] = f;
        return this;
    }

    public FloatIndexer put(long i, long j, float[] f, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n] = f[offset + n];
        }
        return this;
    }

    public FloatIndexer put(long i, long j, long k, float f) {
        this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k)] = f;
        return this;
    }

    public FloatIndexer put(long[] indices, float f) {
        this.array[(int) index(indices)] = f;
        return this;
    }

    public FloatIndexer put(long[] indices, float[] f, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.array[((int) index(indices)) + n] = f[offset + n];
        }
        return this;
    }

    public void release() {
        this.array = null;
    }
}
