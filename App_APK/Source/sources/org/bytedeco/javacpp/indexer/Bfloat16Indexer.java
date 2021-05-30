package org.bytedeco.javacpp.indexer;

import java.nio.ShortBuffer;
import org.bytedeco.javacpp.ShortPointer;
import org.xbill.DNS.TTL;

public abstract class Bfloat16Indexer extends Indexer {
    public static final int VALUE_BYTES = 2;

    public abstract float get(long j);

    public abstract float get(long j, long j2);

    public abstract float get(long j, long j2, long j3);

    public abstract float get(long... jArr);

    public abstract Bfloat16Indexer get(long j, long j2, float[] fArr, int i, int i2);

    public abstract Bfloat16Indexer get(long j, float[] fArr, int i, int i2);

    public abstract Bfloat16Indexer get(long[] jArr, float[] fArr, int i, int i2);

    public abstract Bfloat16Indexer put(long j, float f);

    public abstract Bfloat16Indexer put(long j, long j2, float f);

    public abstract Bfloat16Indexer put(long j, long j2, long j3, float f);

    public abstract Bfloat16Indexer put(long j, long j2, float[] fArr, int i, int i2);

    public abstract Bfloat16Indexer put(long j, float[] fArr, int i, int i2);

    public abstract Bfloat16Indexer put(long[] jArr, float f);

    public abstract Bfloat16Indexer put(long[] jArr, float[] fArr, int i, int i2);

    protected Bfloat16Indexer(long[] sizes, long[] strides) {
        super(sizes, strides);
    }

    public static Bfloat16Indexer create(short[] array) {
        return new Bfloat16ArrayIndexer(array);
    }

    public static Bfloat16Indexer create(ShortBuffer buffer) {
        return new Bfloat16BufferIndexer(buffer);
    }

    public static Bfloat16Indexer create(ShortPointer pointer) {
        return create(pointer, new long[]{pointer.limit() - pointer.position()}, ONE_STRIDE);
    }

    public static Bfloat16Indexer create(short[] array, long[] sizes, long[] strides) {
        return new Bfloat16ArrayIndexer(array, sizes, strides);
    }

    public static Bfloat16Indexer create(ShortBuffer buffer, long[] sizes, long[] strides) {
        return new Bfloat16BufferIndexer(buffer, sizes, strides);
    }

    public static Bfloat16Indexer create(ShortPointer pointer, long[] sizes, long[] strides) {
        return create(pointer, sizes, strides, true);
    }

    public static Bfloat16Indexer create(ShortPointer pointer, long[] sizes, long[] strides, boolean direct) {
        if (!direct) {
            long position = pointer.position();
            short[] array = new short[((int) Math.min(pointer.limit() - position, TTL.MAX_VALUE))];
            pointer.get(array);
            final ShortPointer shortPointer = pointer;
            final long j = position;
            return new Bfloat16ArrayIndexer(array, sizes, strides) {
                public void release() {
                    shortPointer.position(j).put(this.array);
                    super.release();
                }
            };
        } else if (Raw.getInstance() != null) {
            return new Bfloat16RawIndexer(pointer, sizes, strides);
        } else {
            return new Bfloat16BufferIndexer(pointer.asBuffer(), sizes, strides);
        }
    }

    public static float toFloat(int h) {
        return Float.intBitsToFloat(h << 16);
    }

    public static int fromFloat(float h) {
        return Float.floatToIntBits(h) >>> 16;
    }

    public Bfloat16Indexer get(long i, float[] h) {
        return get(i, h, 0, h.length);
    }

    public Bfloat16Indexer get(long i, long j, float[] h) {
        return get(i, j, h, 0, h.length);
    }

    public Bfloat16Indexer get(long[] indices, float[] h) {
        return get(indices, h, 0, h.length);
    }

    public Bfloat16Indexer put(long i, float... h) {
        return put(i, h, 0, h.length);
    }

    public Bfloat16Indexer put(long i, long j, float... h) {
        return put(i, j, h, 0, h.length);
    }

    public Bfloat16Indexer put(long[] indices, float... h) {
        return put(indices, h, 0, h.length);
    }

    public double getDouble(long... indices) {
        return (double) get(indices);
    }

    public Bfloat16Indexer putDouble(long[] indices, double s) {
        return put(indices, (float) s);
    }
}
