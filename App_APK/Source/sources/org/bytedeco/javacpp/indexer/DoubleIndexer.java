package org.bytedeco.javacpp.indexer;

import java.nio.DoubleBuffer;
import org.bytedeco.javacpp.DoublePointer;
import org.xbill.DNS.TTL;

public abstract class DoubleIndexer extends Indexer {
    public static final int VALUE_BYTES = 8;

    public abstract double get(long j);

    public abstract double get(long j, long j2);

    public abstract double get(long j, long j2, long j3);

    public abstract double get(long... jArr);

    public abstract DoubleIndexer get(long j, long j2, double[] dArr, int i, int i2);

    public abstract DoubleIndexer get(long j, double[] dArr, int i, int i2);

    public abstract DoubleIndexer get(long[] jArr, double[] dArr, int i, int i2);

    public abstract DoubleIndexer put(long j, double d);

    public abstract DoubleIndexer put(long j, long j2, double d);

    public abstract DoubleIndexer put(long j, long j2, long j3, double d);

    public abstract DoubleIndexer put(long j, long j2, double[] dArr, int i, int i2);

    public abstract DoubleIndexer put(long j, double[] dArr, int i, int i2);

    public abstract DoubleIndexer put(long[] jArr, double d);

    public abstract DoubleIndexer put(long[] jArr, double[] dArr, int i, int i2);

    protected DoubleIndexer(long[] sizes, long[] strides) {
        super(sizes, strides);
    }

    public static DoubleIndexer create(double[] array) {
        return new DoubleArrayIndexer(array);
    }

    public static DoubleIndexer create(DoubleBuffer buffer) {
        return new DoubleBufferIndexer(buffer);
    }

    public static DoubleIndexer create(DoublePointer pointer) {
        return create(pointer, new long[]{pointer.limit() - pointer.position()}, ONE_STRIDE);
    }

    public static DoubleIndexer create(double[] array, long[] sizes, long[] strides) {
        return new DoubleArrayIndexer(array, sizes, strides);
    }

    public static DoubleIndexer create(DoubleBuffer buffer, long[] sizes, long[] strides) {
        return new DoubleBufferIndexer(buffer, sizes, strides);
    }

    public static DoubleIndexer create(DoublePointer pointer, long[] sizes, long[] strides) {
        return create(pointer, sizes, strides, true);
    }

    public static DoubleIndexer create(DoublePointer pointer, long[] sizes, long[] strides, boolean direct) {
        if (!direct) {
            long position = pointer.position();
            double[] array = new double[((int) Math.min(pointer.limit() - position, TTL.MAX_VALUE))];
            pointer.get(array);
            final DoublePointer doublePointer = pointer;
            final long j = position;
            return new DoubleArrayIndexer(array, sizes, strides) {
                public void release() {
                    doublePointer.position(j).put(this.array);
                    super.release();
                }
            };
        } else if (Raw.getInstance() != null) {
            return new DoubleRawIndexer(pointer, sizes, strides);
        } else {
            return new DoubleBufferIndexer(pointer.asBuffer(), sizes, strides);
        }
    }

    public DoubleIndexer get(long i, double[] d) {
        return get(i, d, 0, d.length);
    }

    public DoubleIndexer get(long i, long j, double[] d) {
        return get(i, j, d, 0, d.length);
    }

    public DoubleIndexer get(long[] indices, double[] d) {
        return get(indices, d, 0, d.length);
    }

    public DoubleIndexer put(long i, double... d) {
        return put(i, d, 0, d.length);
    }

    public DoubleIndexer put(long i, long j, double... d) {
        return put(i, j, d, 0, d.length);
    }

    public DoubleIndexer put(long[] indices, double... d) {
        return put(indices, d, 0, d.length);
    }

    public double getDouble(long... indices) {
        return get(indices);
    }

    public DoubleIndexer putDouble(long[] indices, double d) {
        return put(indices, d);
    }
}
