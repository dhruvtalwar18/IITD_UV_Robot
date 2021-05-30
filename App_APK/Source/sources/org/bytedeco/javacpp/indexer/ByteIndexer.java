package org.bytedeco.javacpp.indexer;

import java.nio.ByteBuffer;
import org.bytedeco.javacpp.BytePointer;
import org.xbill.DNS.TTL;

public abstract class ByteIndexer extends Indexer {
    public static final int VALUE_BYTES = 1;

    public abstract byte get(long j);

    public abstract byte get(long j, long j2);

    public abstract byte get(long j, long j2, long j3);

    public abstract byte get(long... jArr);

    public abstract ByteIndexer get(long j, long j2, byte[] bArr, int i, int i2);

    public abstract ByteIndexer get(long j, byte[] bArr, int i, int i2);

    public abstract ByteIndexer get(long[] jArr, byte[] bArr, int i, int i2);

    public abstract char getChar(long j);

    public abstract double getDouble(long j);

    public abstract float getFloat(long j);

    public abstract int getInt(long j);

    public abstract long getLong(long j);

    public abstract short getShort(long j);

    public abstract ByteIndexer put(long j, byte b);

    public abstract ByteIndexer put(long j, long j2, byte b);

    public abstract ByteIndexer put(long j, long j2, long j3, byte b);

    public abstract ByteIndexer put(long j, long j2, byte[] bArr, int i, int i2);

    public abstract ByteIndexer put(long j, byte[] bArr, int i, int i2);

    public abstract ByteIndexer put(long[] jArr, byte b);

    public abstract ByteIndexer put(long[] jArr, byte[] bArr, int i, int i2);

    public abstract ByteIndexer putChar(long j, char c);

    public abstract ByteIndexer putDouble(long j, double d);

    public abstract ByteIndexer putFloat(long j, float f);

    public abstract ByteIndexer putInt(long j, int i);

    public abstract ByteIndexer putLong(long j, long j2);

    public abstract ByteIndexer putShort(long j, short s);

    protected ByteIndexer(long[] sizes, long[] strides) {
        super(sizes, strides);
    }

    public static ByteIndexer create(byte[] array) {
        return new ByteArrayIndexer(array);
    }

    public static ByteIndexer create(ByteBuffer buffer) {
        return new ByteBufferIndexer(buffer);
    }

    public static ByteIndexer create(BytePointer pointer) {
        return create(pointer, new long[]{pointer.limit() - pointer.position()}, ONE_STRIDE);
    }

    public static ByteIndexer create(byte[] array, long[] sizes, long[] strides) {
        return new ByteArrayIndexer(array, sizes, strides);
    }

    public static ByteIndexer create(ByteBuffer buffer, long[] sizes, long[] strides) {
        return new ByteBufferIndexer(buffer, sizes, strides);
    }

    public static ByteIndexer create(BytePointer pointer, long[] sizes, long[] strides) {
        return create(pointer, sizes, strides, true);
    }

    public static ByteIndexer create(BytePointer pointer, long[] sizes, long[] strides, boolean direct) {
        if (!direct) {
            long position = pointer.position();
            byte[] array = new byte[((int) Math.min(pointer.limit() - position, TTL.MAX_VALUE))];
            pointer.get(array);
            final BytePointer bytePointer = pointer;
            final long j = position;
            return new ByteArrayIndexer(array, sizes, strides) {
                public void release() {
                    bytePointer.position(j).put(this.array);
                    super.release();
                }
            };
        } else if (Raw.getInstance() != null) {
            return new ByteRawIndexer(pointer, sizes, strides);
        } else {
            return new ByteBufferIndexer(pointer.asBuffer(), sizes, strides);
        }
    }

    public ByteIndexer get(long i, byte[] b) {
        return get(i, b, 0, b.length);
    }

    public ByteIndexer get(long i, long j, byte[] b) {
        return get(i, j, b, 0, b.length);
    }

    public ByteIndexer get(long[] indices, byte[] b) {
        return get(indices, b, 0, b.length);
    }

    public ByteIndexer put(long i, byte... b) {
        return put(i, b, 0, b.length);
    }

    public ByteIndexer put(long i, long j, byte... b) {
        return put(i, j, b, 0, b.length);
    }

    public ByteIndexer put(long[] indices, byte... b) {
        return put(indices, b, 0, b.length);
    }

    public double getDouble(long... indices) {
        return (double) get(indices);
    }

    public ByteIndexer putDouble(long[] indices, double b) {
        return put(indices, (byte) ((int) b));
    }
}
