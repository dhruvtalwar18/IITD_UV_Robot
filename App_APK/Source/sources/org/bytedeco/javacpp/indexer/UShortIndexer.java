package org.bytedeco.javacpp.indexer;

import java.nio.ShortBuffer;
import org.bytedeco.javacpp.ShortPointer;
import org.xbill.DNS.TTL;

public abstract class UShortIndexer extends Indexer {
    public static final int VALUE_BYTES = 2;

    public abstract int get(long j);

    public abstract int get(long j, long j2);

    public abstract int get(long j, long j2, long j3);

    public abstract int get(long... jArr);

    public abstract UShortIndexer get(long j, long j2, int[] iArr, int i, int i2);

    public abstract UShortIndexer get(long j, int[] iArr, int i, int i2);

    public abstract UShortIndexer get(long[] jArr, int[] iArr, int i, int i2);

    public abstract UShortIndexer put(long j, int i);

    public abstract UShortIndexer put(long j, long j2, int i);

    public abstract UShortIndexer put(long j, long j2, long j3, int i);

    public abstract UShortIndexer put(long j, long j2, int[] iArr, int i, int i2);

    public abstract UShortIndexer put(long j, int[] iArr, int i, int i2);

    public abstract UShortIndexer put(long[] jArr, int i);

    public abstract UShortIndexer put(long[] jArr, int[] iArr, int i, int i2);

    protected UShortIndexer(long[] sizes, long[] strides) {
        super(sizes, strides);
    }

    public static UShortIndexer create(short[] array) {
        return new UShortArrayIndexer(array);
    }

    public static UShortIndexer create(ShortBuffer buffer) {
        return new UShortBufferIndexer(buffer);
    }

    public static UShortIndexer create(ShortPointer pointer) {
        return create(pointer, new long[]{pointer.limit() - pointer.position()}, ONE_STRIDE);
    }

    public static UShortIndexer create(short[] array, long[] sizes, long[] strides) {
        return new UShortArrayIndexer(array, sizes, strides);
    }

    public static UShortIndexer create(ShortBuffer buffer, long[] sizes, long[] strides) {
        return new UShortBufferIndexer(buffer, sizes, strides);
    }

    public static UShortIndexer create(ShortPointer pointer, long[] sizes, long[] strides) {
        return create(pointer, sizes, strides, true);
    }

    public static UShortIndexer create(ShortPointer pointer, long[] sizes, long[] strides, boolean direct) {
        if (!direct) {
            long position = pointer.position();
            short[] array = new short[((int) Math.min(pointer.limit() - position, TTL.MAX_VALUE))];
            pointer.get(array);
            final ShortPointer shortPointer = pointer;
            final long j = position;
            return new UShortArrayIndexer(array, sizes, strides) {
                public void release() {
                    shortPointer.position(j).put(this.array);
                    super.release();
                }
            };
        } else if (Raw.getInstance() != null) {
            return new UShortRawIndexer(pointer, sizes, strides);
        } else {
            return new UShortBufferIndexer(pointer.asBuffer(), sizes, strides);
        }
    }

    public UShortIndexer get(long i, int[] s) {
        return get(i, s, 0, s.length);
    }

    public UShortIndexer get(long i, long j, int[] s) {
        return get(i, j, s, 0, s.length);
    }

    public UShortIndexer get(long[] indices, int[] s) {
        return get(indices, s, 0, s.length);
    }

    public UShortIndexer put(long i, int... s) {
        return put(i, s, 0, s.length);
    }

    public UShortIndexer put(long i, long j, int... s) {
        return put(i, j, s, 0, s.length);
    }

    public UShortIndexer put(long[] indices, int... s) {
        return put(indices, s, 0, s.length);
    }

    public double getDouble(long... indices) {
        return (double) get(indices);
    }

    public UShortIndexer putDouble(long[] indices, double s) {
        return put(indices, (int) s);
    }
}
