package org.bytedeco.javacpp.indexer;

import java.nio.ShortBuffer;
import org.bytedeco.javacpp.ShortPointer;
import org.xbill.DNS.TTL;

public abstract class ShortIndexer extends Indexer {
    public static final int VALUE_BYTES = 2;

    public abstract ShortIndexer get(long j, long j2, short[] sArr, int i, int i2);

    public abstract ShortIndexer get(long j, short[] sArr, int i, int i2);

    public abstract ShortIndexer get(long[] jArr, short[] sArr, int i, int i2);

    public abstract short get(long j);

    public abstract short get(long j, long j2);

    public abstract short get(long j, long j2, long j3);

    public abstract short get(long... jArr);

    public abstract ShortIndexer put(long j, long j2, long j3, short s);

    public abstract ShortIndexer put(long j, long j2, short s);

    public abstract ShortIndexer put(long j, long j2, short[] sArr, int i, int i2);

    public abstract ShortIndexer put(long j, short s);

    public abstract ShortIndexer put(long j, short[] sArr, int i, int i2);

    public abstract ShortIndexer put(long[] jArr, short s);

    public abstract ShortIndexer put(long[] jArr, short[] sArr, int i, int i2);

    protected ShortIndexer(long[] sizes, long[] strides) {
        super(sizes, strides);
    }

    public static ShortIndexer create(short[] array) {
        return new ShortArrayIndexer(array);
    }

    public static ShortIndexer create(ShortBuffer buffer) {
        return new ShortBufferIndexer(buffer);
    }

    public static ShortIndexer create(ShortPointer pointer) {
        return create(pointer, new long[]{pointer.limit() - pointer.position()}, ONE_STRIDE);
    }

    public static ShortIndexer create(short[] array, long[] sizes, long[] strides) {
        return new ShortArrayIndexer(array, sizes, strides);
    }

    public static ShortIndexer create(ShortBuffer buffer, long[] sizes, long[] strides) {
        return new ShortBufferIndexer(buffer, sizes, strides);
    }

    public static ShortIndexer create(ShortPointer pointer, long[] sizes, long[] strides) {
        return create(pointer, sizes, strides, true);
    }

    public static ShortIndexer create(ShortPointer pointer, long[] sizes, long[] strides, boolean direct) {
        if (!direct) {
            long position = pointer.position();
            short[] array = new short[((int) Math.min(pointer.limit() - position, TTL.MAX_VALUE))];
            pointer.get(array);
            final ShortPointer shortPointer = pointer;
            final long j = position;
            return new ShortArrayIndexer(array, sizes, strides) {
                public void release() {
                    shortPointer.position(j).put(this.array);
                    super.release();
                }
            };
        } else if (Raw.getInstance() != null) {
            return new ShortRawIndexer(pointer, sizes, strides);
        } else {
            return new ShortBufferIndexer(pointer.asBuffer(), sizes, strides);
        }
    }

    public ShortIndexer get(long i, short[] s) {
        return get(i, s, 0, s.length);
    }

    public ShortIndexer get(long i, long j, short[] s) {
        return get(i, j, s, 0, s.length);
    }

    public ShortIndexer get(long[] indices, short[] s) {
        return get(indices, s, 0, s.length);
    }

    public ShortIndexer put(long i, short... s) {
        return put(i, s, 0, s.length);
    }

    public ShortIndexer put(long i, long j, short... s) {
        return put(i, j, s, 0, s.length);
    }

    public ShortIndexer put(long[] indices, short... s) {
        return put(indices, s, 0, s.length);
    }

    public double getDouble(long... indices) {
        return (double) get(indices);
    }

    public ShortIndexer putDouble(long[] indices, double s) {
        return put(indices, (short) ((int) s));
    }
}
