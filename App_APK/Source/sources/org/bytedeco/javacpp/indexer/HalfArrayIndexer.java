package org.bytedeco.javacpp.indexer;

public class HalfArrayIndexer extends HalfIndexer {
    protected short[] array;

    public /* bridge */ /* synthetic */ Indexer putDouble(long[] jArr, double d) {
        return super.putDouble(jArr, d);
    }

    public HalfArrayIndexer(short[] array2) {
        this(array2, new long[]{(long) array2.length}, ONE_STRIDE);
    }

    public HalfArrayIndexer(short[] array2, long[] sizes, long[] strides) {
        super(sizes, strides);
        this.array = array2;
    }

    public short[] array() {
        return this.array;
    }

    public float get(long i) {
        return toFloat(this.array[(int) i]);
    }

    public HalfIndexer get(long i, float[] h, int offset, int length) {
        for (int n = 0; n < length; n++) {
            h[offset + n] = toFloat(this.array[(((int) i) * ((int) this.strides[0])) + n]);
        }
        return this;
    }

    public float get(long i, long j) {
        return toFloat(this.array[(((int) i) * ((int) this.strides[0])) + ((int) j)]);
    }

    public HalfIndexer get(long i, long j, float[] h, int offset, int length) {
        for (int n = 0; n < length; n++) {
            h[offset + n] = toFloat(this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n]);
        }
        long j2 = i;
        long j3 = j;
        return this;
    }

    public float get(long i, long j, long k) {
        return toFloat(this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k)]);
    }

    public float get(long... indices) {
        return toFloat(this.array[(int) index(indices)]);
    }

    public HalfIndexer get(long[] indices, float[] h, int offset, int length) {
        for (int n = 0; n < length; n++) {
            h[offset + n] = toFloat(this.array[((int) index(indices)) + n]);
        }
        return this;
    }

    public HalfIndexer put(long i, float h) {
        this.array[(int) i] = (short) fromFloat(h);
        return this;
    }

    public HalfIndexer put(long i, float[] h, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.array[(((int) i) * ((int) this.strides[0])) + n] = (short) fromFloat(h[offset + n]);
        }
        return this;
    }

    public HalfIndexer put(long i, long j, float h) {
        this.array[(((int) i) * ((int) this.strides[0])) + ((int) j)] = (short) fromFloat(h);
        return this;
    }

    public HalfIndexer put(long i, long j, float[] h, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n] = (short) fromFloat(h[offset + n]);
        }
        return this;
    }

    public HalfIndexer put(long i, long j, long k, float h) {
        this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k)] = (short) fromFloat(h);
        return this;
    }

    public HalfIndexer put(long[] indices, float h) {
        this.array[(int) index(indices)] = (short) fromFloat(h);
        return this;
    }

    public HalfIndexer put(long[] indices, float[] h, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.array[((int) index(indices)) + n] = (short) fromFloat(h[offset + n]);
        }
        return this;
    }

    public void release() {
        this.array = null;
    }
}
