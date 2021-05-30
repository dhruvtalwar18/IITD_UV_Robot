package org.bytedeco.javacpp.indexer;

import java.nio.ShortBuffer;
import org.apache.commons.net.bsd.RCommandClient;
import org.bytedeco.javacpp.ShortPointer;
import org.xbill.DNS.TTL;

public abstract class HalfIndexer extends Indexer {
    public static final int VALUE_BYTES = 2;

    public abstract float get(long j);

    public abstract float get(long j, long j2);

    public abstract float get(long j, long j2, long j3);

    public abstract float get(long... jArr);

    public abstract HalfIndexer get(long j, long j2, float[] fArr, int i, int i2);

    public abstract HalfIndexer get(long j, float[] fArr, int i, int i2);

    public abstract HalfIndexer get(long[] jArr, float[] fArr, int i, int i2);

    public abstract HalfIndexer put(long j, float f);

    public abstract HalfIndexer put(long j, long j2, float f);

    public abstract HalfIndexer put(long j, long j2, long j3, float f);

    public abstract HalfIndexer put(long j, long j2, float[] fArr, int i, int i2);

    public abstract HalfIndexer put(long j, float[] fArr, int i, int i2);

    public abstract HalfIndexer put(long[] jArr, float f);

    public abstract HalfIndexer put(long[] jArr, float[] fArr, int i, int i2);

    protected HalfIndexer(long[] sizes, long[] strides) {
        super(sizes, strides);
    }

    public static HalfIndexer create(short[] array) {
        return new HalfArrayIndexer(array);
    }

    public static HalfIndexer create(ShortBuffer buffer) {
        return new HalfBufferIndexer(buffer);
    }

    public static HalfIndexer create(ShortPointer pointer) {
        return create(pointer, new long[]{pointer.limit() - pointer.position()}, ONE_STRIDE);
    }

    public static HalfIndexer create(short[] array, long[] sizes, long[] strides) {
        return new HalfArrayIndexer(array, sizes, strides);
    }

    public static HalfIndexer create(ShortBuffer buffer, long[] sizes, long[] strides) {
        return new HalfBufferIndexer(buffer, sizes, strides);
    }

    public static HalfIndexer create(ShortPointer pointer, long[] sizes, long[] strides) {
        return create(pointer, sizes, strides, true);
    }

    public static HalfIndexer create(ShortPointer pointer, long[] sizes, long[] strides, boolean direct) {
        if (!direct) {
            long position = pointer.position();
            short[] array = new short[((int) Math.min(pointer.limit() - position, TTL.MAX_VALUE))];
            pointer.get(array);
            final ShortPointer shortPointer = pointer;
            final long j = position;
            return new HalfArrayIndexer(array, sizes, strides) {
                public void release() {
                    shortPointer.position(j).put(this.array);
                    super.release();
                }
            };
        } else if (Raw.getInstance() != null) {
            return new HalfRawIndexer(pointer, sizes, strides);
        } else {
            return new HalfBufferIndexer(pointer.asBuffer(), sizes, strides);
        }
    }

    public static float toFloat(int hbits) {
        int mant = hbits & RCommandClient.MAX_CLIENT_PORT;
        int exp = hbits & 31744;
        if (exp == 31744) {
            exp = 261120;
        } else if (exp != 0) {
            exp += 114688;
        } else if (mant != 0) {
            exp = 115712;
            do {
                mant <<= 1;
                exp -= 1024;
            } while ((mant & 1024) == 0);
            mant &= RCommandClient.MAX_CLIENT_PORT;
        }
        return Float.intBitsToFloat(((32768 & hbits) << 16) | ((exp | mant) << 13));
    }

    public static int fromFloat(float fval) {
        int fbits = Float.floatToIntBits(fval);
        int sign = (fbits >>> 16) & 32768;
        int val = (fbits & Integer.MAX_VALUE) + 4096;
        if (val >= 1199570944) {
            if ((Integer.MAX_VALUE & fbits) < 1199570944) {
                return sign | 31743;
            }
            if (val < 2139095040) {
                return sign | 31744;
            }
            return sign | 31744 | ((8388607 & fbits) >>> 13);
        } else if (val >= 947912704) {
            return ((val - 939524096) >>> 13) | sign;
        } else {
            if (val < 855638016) {
                return sign;
            }
            int val2 = (Integer.MAX_VALUE & fbits) >>> 23;
            return ((((fbits & 8388607) | 8388608) + (8388608 >>> (val2 - 102))) >>> (126 - val2)) | sign;
        }
    }

    public HalfIndexer get(long i, float[] h) {
        return get(i, h, 0, h.length);
    }

    public HalfIndexer get(long i, long j, float[] h) {
        return get(i, j, h, 0, h.length);
    }

    public HalfIndexer get(long[] indices, float[] h) {
        return get(indices, h, 0, h.length);
    }

    public HalfIndexer put(long i, float... h) {
        return put(i, h, 0, h.length);
    }

    public HalfIndexer put(long i, long j, float... h) {
        return put(i, j, h, 0, h.length);
    }

    public HalfIndexer put(long[] indices, float... h) {
        return put(indices, h, 0, h.length);
    }

    public double getDouble(long... indices) {
        return (double) get(indices);
    }

    public HalfIndexer putDouble(long[] indices, double s) {
        return put(indices, (float) s);
    }
}
