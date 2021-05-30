package org.bytedeco.javacpp.indexer;

import java.nio.ByteBuffer;
import org.bytedeco.javacpp.BooleanPointer;
import org.bytedeco.javacpp.opencv_stitching;
import org.xbill.DNS.TTL;

public abstract class BooleanIndexer extends Indexer {
    public static final int VALUE_BYTES = 1;

    public abstract BooleanIndexer get(long j, long j2, boolean[] zArr, int i, int i2);

    public abstract BooleanIndexer get(long j, boolean[] zArr, int i, int i2);

    public abstract BooleanIndexer get(long[] jArr, boolean[] zArr, int i, int i2);

    public abstract boolean get(long j);

    public abstract boolean get(long j, long j2);

    public abstract boolean get(long j, long j2, long j3);

    public abstract boolean get(long... jArr);

    public abstract BooleanIndexer put(long j, long j2, long j3, boolean z);

    public abstract BooleanIndexer put(long j, long j2, boolean z);

    public abstract BooleanIndexer put(long j, long j2, boolean[] zArr, int i, int i2);

    public abstract BooleanIndexer put(long j, boolean z);

    public abstract BooleanIndexer put(long j, boolean[] zArr, int i, int i2);

    public abstract BooleanIndexer put(long[] jArr, boolean z);

    public abstract BooleanIndexer put(long[] jArr, boolean[] zArr, int i, int i2);

    protected BooleanIndexer(long[] sizes, long[] strides) {
        super(sizes, strides);
    }

    public static BooleanIndexer create(boolean[] array) {
        return new BooleanArrayIndexer(array);
    }

    public static BooleanIndexer create(ByteBuffer buffer) {
        return new BooleanBufferIndexer(buffer);
    }

    public static BooleanIndexer create(BooleanPointer pointer) {
        return create(pointer, new long[]{pointer.limit() - pointer.position()}, ONE_STRIDE);
    }

    public static BooleanIndexer create(boolean[] array, long[] sizes, long[] strides) {
        return new BooleanArrayIndexer(array, sizes, strides);
    }

    public static BooleanIndexer create(ByteBuffer buffer, long[] sizes, long[] strides) {
        return new BooleanBufferIndexer(buffer, sizes, strides);
    }

    public static BooleanIndexer create(BooleanPointer pointer, long[] sizes, long[] strides) {
        return create(pointer, sizes, strides, true);
    }

    public static BooleanIndexer create(BooleanPointer pointer, long[] sizes, long[] strides, boolean direct) {
        if (!direct) {
            long position = pointer.position();
            boolean[] array = new boolean[((int) Math.min(pointer.limit() - position, TTL.MAX_VALUE))];
            pointer.get(array);
            final BooleanPointer booleanPointer = pointer;
            final long j = position;
            return new BooleanArrayIndexer(array, sizes, strides) {
                public void release() {
                    booleanPointer.position(j).put(this.array);
                    super.release();
                }
            };
        } else if (Raw.getInstance() != null) {
            return new BooleanRawIndexer(pointer, sizes, strides);
        } else {
            return new BooleanBufferIndexer(pointer.asByteBuffer(), sizes, strides);
        }
    }

    public BooleanIndexer get(long i, boolean[] b) {
        return get(i, b, 0, b.length);
    }

    public BooleanIndexer get(long i, long j, boolean[] b) {
        return get(i, j, b, 0, b.length);
    }

    public BooleanIndexer get(long[] indices, boolean[] b) {
        return get(indices, b, 0, b.length);
    }

    public BooleanIndexer put(long i, boolean... b) {
        return put(i, b, 0, b.length);
    }

    public BooleanIndexer put(long i, long j, boolean... b) {
        return put(i, j, b, 0, b.length);
    }

    public BooleanIndexer put(long[] indices, boolean... b) {
        return put(indices, b, 0, b.length);
    }

    public double getDouble(long... indices) {
        if (get(indices)) {
            return 1.0d;
        }
        return opencv_stitching.Stitcher.ORIG_RESOL;
    }

    public BooleanIndexer putDouble(long[] indices, double b) {
        return put(indices, b != opencv_stitching.Stitcher.ORIG_RESOL);
    }
}
