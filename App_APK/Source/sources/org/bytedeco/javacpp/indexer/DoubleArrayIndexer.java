package org.bytedeco.javacpp.indexer;

public class DoubleArrayIndexer extends DoubleIndexer {
    protected double[] array;

    public /* bridge */ /* synthetic */ Indexer putDouble(long[] jArr, double d) {
        return super.putDouble(jArr, d);
    }

    public DoubleArrayIndexer(double[] array2) {
        this(array2, new long[]{(long) array2.length}, ONE_STRIDE);
    }

    public DoubleArrayIndexer(double[] array2, long[] sizes, long[] strides) {
        super(sizes, strides);
        this.array = array2;
    }

    public double[] array() {
        return this.array;
    }

    public double get(long i) {
        return this.array[(int) i];
    }

    public DoubleIndexer get(long i, double[] d, int offset, int length) {
        for (int n = 0; n < length; n++) {
            d[offset + n] = this.array[(((int) i) * ((int) this.strides[0])) + n];
        }
        return this;
    }

    public double get(long i, long j) {
        return this.array[(((int) i) * ((int) this.strides[0])) + ((int) j)];
    }

    public DoubleIndexer get(long i, long j, double[] d, int offset, int length) {
        for (int n = 0; n < length; n++) {
            d[offset + n] = this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n];
        }
        long j2 = i;
        long j3 = j;
        return this;
    }

    public double get(long i, long j, long k) {
        return this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k)];
    }

    public double get(long... indices) {
        return this.array[(int) index(indices)];
    }

    public DoubleIndexer get(long[] indices, double[] d, int offset, int length) {
        for (int n = 0; n < length; n++) {
            d[offset + n] = this.array[((int) index(indices)) + n];
        }
        return this;
    }

    public DoubleIndexer put(long i, double d) {
        this.array[(int) i] = d;
        return this;
    }

    public DoubleIndexer put(long i, double[] d, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.array[(((int) i) * ((int) this.strides[0])) + n] = d[offset + n];
        }
        return this;
    }

    public DoubleIndexer put(long i, long j, double d) {
        this.array[(((int) i) * ((int) this.strides[0])) + ((int) j)] = d;
        return this;
    }

    public DoubleIndexer put(long i, long j, double[] d, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n] = d[offset + n];
        }
        return this;
    }

    public DoubleIndexer put(long i, long j, long k, double d) {
        this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k)] = d;
        return this;
    }

    public DoubleIndexer put(long[] indices, double d) {
        this.array[(int) index(indices)] = d;
        return this;
    }

    public DoubleIndexer put(long[] indices, double[] d, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.array[((int) index(indices)) + n] = d[offset + n];
        }
        return this;
    }

    public void release() {
        this.array = null;
    }
}
