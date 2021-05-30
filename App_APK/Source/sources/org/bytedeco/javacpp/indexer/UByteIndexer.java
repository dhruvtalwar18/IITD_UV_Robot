package org.bytedeco.javacpp.indexer;

import java.nio.ByteBuffer;
import org.bytedeco.javacpp.BytePointer;
import org.xbill.DNS.TTL;

public abstract class UByteIndexer extends Indexer {
    public static final int VALUE_BYTES = 1;

    public abstract int get(long j);

    public abstract int get(long j, long j2);

    public abstract int get(long j, long j2, long j3);

    public abstract int get(long... jArr);

    public abstract UByteIndexer get(long j, long j2, int[] iArr, int i, int i2);

    public abstract UByteIndexer get(long j, int[] iArr, int i, int i2);

    public abstract UByteIndexer get(long[] jArr, int[] iArr, int i, int i2);

    public abstract UByteIndexer put(long j, int i);

    public abstract UByteIndexer put(long j, long j2, int i);

    public abstract UByteIndexer put(long j, long j2, long j3, int i);

    public abstract UByteIndexer put(long j, long j2, int[] iArr, int i, int i2);

    public abstract UByteIndexer put(long j, int[] iArr, int i, int i2);

    public abstract UByteIndexer put(long[] jArr, int i);

    public abstract UByteIndexer put(long[] jArr, int[] iArr, int i, int i2);

    protected UByteIndexer(long[] sizes, long[] strides) {
        super(sizes, strides);
    }

    public static UByteIndexer create(byte[] array) {
        return new UByteArrayIndexer(array);
    }

    public static UByteIndexer create(ByteBuffer buffer) {
        return new UByteBufferIndexer(buffer);
    }

    public static UByteIndexer create(BytePointer pointer) {
        return create(pointer, new long[]{pointer.limit() - pointer.position()}, ONE_STRIDE);
    }

    public static UByteIndexer create(byte[] array, long[] sizes, long[] strides) {
        return new UByteArrayIndexer(array, sizes, strides);
    }

    public static UByteIndexer create(ByteBuffer buffer, long[] sizes, long[] strides) {
        return new UByteBufferIndexer(buffer, sizes, strides);
    }

    public static UByteIndexer create(BytePointer pointer, long[] sizes, long[] strides) {
        return create(pointer, sizes, strides, true);
    }

    public static UByteIndexer create(BytePointer pointer, long[] sizes, long[] strides, boolean direct) {
        if (!direct) {
            long position = pointer.position();
            byte[] array = new byte[((int) Math.min(pointer.limit() - position, TTL.MAX_VALUE))];
            pointer.get(array);
            final BytePointer bytePointer = pointer;
            final long j = position;
            return new UByteArrayIndexer(array, sizes, strides) {
                public void release() {
                    bytePointer.position(j).put(this.array);
                    super.release();
                }
            };
        } else if (Raw.getInstance() != null) {
            return new UByteRawIndexer(pointer, sizes, strides);
        } else {
            return new UByteBufferIndexer(pointer.asBuffer(), sizes, strides);
        }
    }

    public UByteIndexer get(long i, int[] b) {
        return get(i, b, 0, b.length);
    }

    public UByteIndexer get(long i, long j, int[] b) {
        return get(i, j, b, 0, b.length);
    }

    public UByteIndexer get(long[] indices, int[] b) {
        return get(indices, b, 0, b.length);
    }

    public UByteIndexer put(long i, int... b) {
        return put(i, b, 0, b.length);
    }

    public UByteIndexer put(long i, long j, int... b) {
        return put(i, j, b, 0, b.length);
    }

    public UByteIndexer put(long[] indices, int... b) {
        return put(indices, b, 0, b.length);
    }

    public double getDouble(long... indices) {
        return (double) get(indices);
    }

    public UByteIndexer putDouble(long[] indices, double b) {
        return put(indices, (int) b);
    }
}
