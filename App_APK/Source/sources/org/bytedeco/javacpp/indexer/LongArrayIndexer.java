package org.bytedeco.javacpp.indexer;

public class LongArrayIndexer extends LongIndexer {
    protected long[] array;

    public /* bridge */ /* synthetic */ Indexer putDouble(long[] jArr, double d) {
        return super.putDouble(jArr, d);
    }

    public LongArrayIndexer(long[] array2) {
        this(array2, new long[]{(long) array2.length}, ONE_STRIDE);
    }

    public LongArrayIndexer(long[] array2, long[] sizes, long[] strides) {
        super(sizes, strides);
        this.array = array2;
    }

    public long[] array() {
        return this.array;
    }

    public long get(long i) {
        return this.array[(int) i];
    }

    public LongIndexer get(long i, long[] l, int offset, int length) {
        for (int n = 0; n < length; n++) {
            l[offset + n] = this.array[(((int) i) * ((int) this.strides[0])) + n];
        }
        return this;
    }

    public long get(long i, long j) {
        return this.array[(((int) i) * ((int) this.strides[0])) + ((int) j)];
    }

    public LongIndexer get(long i, long j, long[] l, int offset, int length) {
        for (int n = 0; n < length; n++) {
            l[offset + n] = this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n];
        }
        long j2 = i;
        long j3 = j;
        return this;
    }

    public long get(long i, long j, long k) {
        return this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k)];
    }

    public long get(long... indices) {
        return this.array[(int) index(indices)];
    }

    public LongIndexer get(long[] indices, long[] l, int offset, int length) {
        for (int n = 0; n < length; n++) {
            l[offset + n] = this.array[((int) index(indices)) + n];
        }
        return this;
    }

    public LongIndexer put(long i, long l) {
        this.array[(int) i] = l;
        return this;
    }

    public LongIndexer put(long i, long[] l, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.array[(((int) i) * ((int) this.strides[0])) + n] = l[offset + n];
        }
        return this;
    }

    public LongIndexer put(long i, long j, long l) {
        this.array[(((int) i) * ((int) this.strides[0])) + ((int) j)] = l;
        return this;
    }

    public LongIndexer put(long i, long j, long[] l, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n] = l[offset + n];
        }
        return this;
    }

    public LongIndexer put(long i, long j, long k, long l) {
        this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k)] = l;
        return this;
    }

    public LongIndexer put(long[] indices, long l) {
        this.array[(int) index(indices)] = l;
        return this;
    }

    public LongIndexer put(long[] indices, long[] l, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.array[((int) index(indices)) + n] = l[offset + n];
        }
        return this;
    }

    public void release() {
        this.array = null;
    }
}
