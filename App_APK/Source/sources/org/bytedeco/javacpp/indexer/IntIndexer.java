package org.bytedeco.javacpp.indexer;

import java.nio.IntBuffer;
import org.bytedeco.javacpp.IntPointer;
import org.xbill.DNS.TTL;

public abstract class IntIndexer extends Indexer {
    public static final int VALUE_BYTES = 4;

    public abstract int get(long j);

    public abstract int get(long j, long j2);

    public abstract int get(long j, long j2, long j3);

    public abstract int get(long... jArr);

    public abstract IntIndexer get(long j, long j2, int[] iArr, int i, int i2);

    public abstract IntIndexer get(long j, int[] iArr, int i, int i2);

    public abstract IntIndexer get(long[] jArr, int[] iArr, int i, int i2);

    public abstract IntIndexer put(long j, int i);

    public abstract IntIndexer put(long j, long j2, int i);

    public abstract IntIndexer put(long j, long j2, long j3, int i);

    public abstract IntIndexer put(long j, long j2, int[] iArr, int i, int i2);

    public abstract IntIndexer put(long j, int[] iArr, int i, int i2);

    public abstract IntIndexer put(long[] jArr, int i);

    public abstract IntIndexer put(long[] jArr, int[] iArr, int i, int i2);

    protected IntIndexer(long[] sizes, long[] strides) {
        super(sizes, strides);
    }

    public static IntIndexer create(int[] array) {
        return new IntArrayIndexer(array);
    }

    public static IntIndexer create(IntBuffer buffer) {
        return new IntBufferIndexer(buffer);
    }

    public static IntIndexer create(IntPointer pointer) {
        return create(pointer, new long[]{pointer.limit() - pointer.position()}, ONE_STRIDE);
    }

    public static IntIndexer create(int[] array, long[] sizes, long[] strides) {
        return new IntArrayIndexer(array, sizes, strides);
    }

    public static IntIndexer create(IntBuffer buffer, long[] sizes, long[] strides) {
        return new IntBufferIndexer(buffer, sizes, strides);
    }

    public static IntIndexer create(IntPointer pointer, long[] sizes, long[] strides) {
        return create(pointer, sizes, strides, true);
    }

    public static IntIndexer create(IntPointer pointer, long[] sizes, long[] strides, boolean direct) {
        if (!direct) {
            long position = pointer.position();
            int[] array = new int[((int) Math.min(pointer.limit() - position, TTL.MAX_VALUE))];
            pointer.get(array);
            final IntPointer intPointer = pointer;
            final long j = position;
            return new IntArrayIndexer(array, sizes, strides) {
                public void release() {
                    intPointer.position(j).put(this.array);
                    super.release();
                }
            };
        } else if (Raw.getInstance() != null) {
            return new IntRawIndexer(pointer, sizes, strides);
        } else {
            return new IntBufferIndexer(pointer.asBuffer(), sizes, strides);
        }
    }

    public IntIndexer get(long i, int[] n) {
        return get(i, n, 0, n.length);
    }

    public IntIndexer get(long i, long j, int[] n) {
        return get(i, j, n, 0, n.length);
    }

    public IntIndexer get(long[] indices, int[] n) {
        return get(indices, n, 0, n.length);
    }

    public IntIndexer put(long i, int... n) {
        return put(i, n, 0, n.length);
    }

    public IntIndexer put(long i, long j, int... n) {
        return put(i, j, n, 0, n.length);
    }

    public IntIndexer put(long[] indices, int... n) {
        return put(indices, n, 0, n.length);
    }

    public double getDouble(long... indices) {
        return (double) get(indices);
    }

    public IntIndexer putDouble(long[] indices, double n) {
        return put(indices, (int) n);
    }
}
