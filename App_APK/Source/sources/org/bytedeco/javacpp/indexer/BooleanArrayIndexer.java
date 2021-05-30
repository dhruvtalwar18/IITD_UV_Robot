package org.bytedeco.javacpp.indexer;

public class BooleanArrayIndexer extends BooleanIndexer {
    protected boolean[] array;

    public /* bridge */ /* synthetic */ Indexer putDouble(long[] jArr, double d) {
        return super.putDouble(jArr, d);
    }

    public BooleanArrayIndexer(boolean[] array2) {
        this(array2, new long[]{(long) array2.length}, ONE_STRIDE);
    }

    public BooleanArrayIndexer(boolean[] array2, long[] sizes, long[] strides) {
        super(sizes, strides);
        this.array = array2;
    }

    public boolean[] array() {
        return this.array;
    }

    public boolean get(long i) {
        return this.array[(int) i];
    }

    public BooleanIndexer get(long i, boolean[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            b[offset + n] = this.array[(((int) i) * ((int) this.strides[0])) + n];
        }
        return this;
    }

    public boolean get(long i, long j) {
        return this.array[(((int) i) * ((int) this.strides[0])) + ((int) j)];
    }

    public BooleanIndexer get(long i, long j, boolean[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            b[offset + n] = this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n];
        }
        long j2 = i;
        long j3 = j;
        return this;
    }

    public boolean get(long i, long j, long k) {
        return this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k)];
    }

    public boolean get(long... indices) {
        return this.array[(int) index(indices)];
    }

    public BooleanIndexer get(long[] indices, boolean[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            b[offset + n] = this.array[((int) index(indices)) + n];
        }
        return this;
    }

    public BooleanIndexer put(long i, boolean b) {
        this.array[(int) i] = b;
        return this;
    }

    public BooleanIndexer put(long i, boolean[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.array[(((int) i) * ((int) this.strides[0])) + n] = b[offset + n];
        }
        return this;
    }

    public BooleanIndexer put(long i, long j, boolean b) {
        this.array[(((int) i) * ((int) this.strides[0])) + ((int) j)] = b;
        return this;
    }

    public BooleanIndexer put(long i, long j, boolean[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n] = b[offset + n];
        }
        return this;
    }

    public BooleanIndexer put(long i, long j, long k, boolean b) {
        this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k)] = b;
        return this;
    }

    public BooleanIndexer put(long[] indices, boolean b) {
        this.array[(int) index(indices)] = b;
        return this;
    }

    public BooleanIndexer put(long[] indices, boolean[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.array[((int) index(indices)) + n] = b[offset + n];
        }
        return this;
    }

    public void release() {
        this.array = null;
    }
}
