package org.bytedeco.javacpp.indexer;

import java.nio.FloatBuffer;
import org.bytedeco.javacpp.FloatPointer;
import org.xbill.DNS.TTL;

public abstract class FloatIndexer extends Indexer {
    public static final int VALUE_BYTES = 4;

    public abstract float get(long j);

    public abstract float get(long j, long j2);

    public abstract float get(long j, long j2, long j3);

    public abstract float get(long... jArr);

    public abstract FloatIndexer get(long j, long j2, float[] fArr, int i, int i2);

    public abstract FloatIndexer get(long j, float[] fArr, int i, int i2);

    public abstract FloatIndexer get(long[] jArr, float[] fArr, int i, int i2);

    public abstract FloatIndexer put(long j, float f);

    public abstract FloatIndexer put(long j, long j2, float f);

    public abstract FloatIndexer put(long j, long j2, long j3, float f);

    public abstract FloatIndexer put(long j, long j2, float[] fArr, int i, int i2);

    public abstract FloatIndexer put(long j, float[] fArr, int i, int i2);

    public abstract FloatIndexer put(long[] jArr, float f);

    public abstract FloatIndexer put(long[] jArr, float[] fArr, int i, int i2);

    protected FloatIndexer(long[] sizes, long[] strides) {
        super(sizes, strides);
    }

    public static FloatIndexer create(float[] array) {
        return new FloatArrayIndexer(array);
    }

    public static FloatIndexer create(FloatBuffer buffer) {
        return new FloatBufferIndexer(buffer);
    }

    public static FloatIndexer create(FloatPointer pointer) {
        return create(pointer, new long[]{pointer.limit() - pointer.position()}, ONE_STRIDE);
    }

    public static FloatIndexer create(float[] array, long[] sizes, long[] strides) {
        return new FloatArrayIndexer(array, sizes, strides);
    }

    public static FloatIndexer create(FloatBuffer buffer, long[] sizes, long[] strides) {
        return new FloatBufferIndexer(buffer, sizes, strides);
    }

    public static FloatIndexer create(FloatPointer pointer, long[] sizes, long[] strides) {
        return create(pointer, sizes, strides, true);
    }

    public static FloatIndexer create(FloatPointer pointer, long[] sizes, long[] strides, boolean direct) {
        if (!direct) {
            long position = pointer.position();
            float[] array = new float[((int) Math.min(pointer.limit() - position, TTL.MAX_VALUE))];
            pointer.get(array);
            final FloatPointer floatPointer = pointer;
            final long j = position;
            return new FloatArrayIndexer(array, sizes, strides) {
                public void release() {
                    floatPointer.position(j).put(this.array);
                    super.release();
                }
            };
        } else if (Raw.getInstance() != null) {
            return new FloatRawIndexer(pointer, sizes, strides);
        } else {
            return new FloatBufferIndexer(pointer.asBuffer(), sizes, strides);
        }
    }

    public FloatIndexer get(long i, float[] f) {
        return get(i, f, 0, f.length);
    }

    public FloatIndexer get(long i, long j, float[] f) {
        return get(i, j, f, 0, f.length);
    }

    public FloatIndexer get(long[] indices, float[] f) {
        return get(indices, f, 0, f.length);
    }

    public FloatIndexer put(long i, float... f) {
        return put(i, f, 0, f.length);
    }

    public FloatIndexer put(long i, long j, float... f) {
        return put(i, j, f, 0, f.length);
    }

    public FloatIndexer put(long[] indices, float... f) {
        return put(indices, f, 0, f.length);
    }

    public double getDouble(long... indices) {
        return (double) get(indices);
    }

    public FloatIndexer putDouble(long[] indices, double f) {
        return put(indices, (float) f);
    }
}
