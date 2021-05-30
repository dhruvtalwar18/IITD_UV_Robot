package org.bytedeco.javacpp.indexer;

import java.nio.LongBuffer;
import org.bytedeco.javacpp.LongPointer;
import org.xbill.DNS.TTL;

public abstract class LongIndexer extends Indexer {
    public static final int VALUE_BYTES = 8;

    public abstract long get(long j);

    public abstract long get(long j, long j2);

    public abstract long get(long j, long j2, long j3);

    public abstract long get(long... jArr);

    public abstract LongIndexer get(long j, long j2, long[] jArr, int i, int i2);

    public abstract LongIndexer get(long j, long[] jArr, int i, int i2);

    public abstract LongIndexer get(long[] jArr, long[] jArr2, int i, int i2);

    public abstract LongIndexer put(long j, long j2);

    public abstract LongIndexer put(long j, long j2, long j3);

    public abstract LongIndexer put(long j, long j2, long j3, long j4);

    public abstract LongIndexer put(long j, long j2, long[] jArr, int i, int i2);

    public abstract LongIndexer put(long j, long[] jArr, int i, int i2);

    public abstract LongIndexer put(long[] jArr, long j);

    public abstract LongIndexer put(long[] jArr, long[] jArr2, int i, int i2);

    protected LongIndexer(long[] sizes, long[] strides) {
        super(sizes, strides);
    }

    public static LongIndexer create(long[] array) {
        return new LongArrayIndexer(array);
    }

    public static LongIndexer create(LongBuffer buffer) {
        return new LongBufferIndexer(buffer);
    }

    public static LongIndexer create(LongPointer pointer) {
        return create(pointer, new long[]{pointer.limit() - pointer.position()}, ONE_STRIDE);
    }

    public static LongIndexer create(long[] array, long[] sizes, long[] strides) {
        return new LongArrayIndexer(array, sizes, strides);
    }

    public static LongIndexer create(LongBuffer buffer, long[] sizes, long[] strides) {
        return new LongBufferIndexer(buffer, sizes, strides);
    }

    public static LongIndexer create(LongPointer pointer, long[] sizes, long[] strides) {
        return create(pointer, sizes, strides, true);
    }

    public static LongIndexer create(LongPointer pointer, long[] sizes, long[] strides, boolean direct) {
        if (!direct) {
            long position = pointer.position();
            long[] array = new long[((int) Math.min(pointer.limit() - position, TTL.MAX_VALUE))];
            pointer.get(array);
            final LongPointer longPointer = pointer;
            final long j = position;
            return new LongArrayIndexer(array, sizes, strides) {
                public void release() {
                    longPointer.position(j).put(this.array);
                    super.release();
                }
            };
        } else if (Raw.getInstance() != null) {
            return new LongRawIndexer(pointer, sizes, strides);
        } else {
            return new LongBufferIndexer(pointer.asBuffer(), sizes, strides);
        }
    }

    public LongIndexer get(long i, long[] l) {
        return get(i, l, 0, l.length);
    }

    public LongIndexer get(long i, long j, long[] l) {
        return get(i, j, l, 0, l.length);
    }

    public LongIndexer get(long[] indices, long[] l) {
        return get(indices, l, 0, l.length);
    }

    public LongIndexer put(long i, long... l) {
        return put(i, l, 0, l.length);
    }

    public LongIndexer put(long i, long j, long... l) {
        return put(i, j, l, 0, l.length);
    }

    public LongIndexer put(long[] indices, long... l) {
        return put(indices, l, 0, l.length);
    }

    public double getDouble(long... indices) {
        return (double) get(indices);
    }

    public LongIndexer putDouble(long[] indices, double l) {
        return put(indices, (long) l);
    }
}
