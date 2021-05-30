package org.bytedeco.javacpp.indexer;

import java.nio.CharBuffer;
import org.bytedeco.javacpp.CharPointer;
import org.xbill.DNS.TTL;

public abstract class CharIndexer extends Indexer {
    public static final int VALUE_BYTES = 2;

    public abstract char get(long j);

    public abstract char get(long j, long j2);

    public abstract char get(long j, long j2, long j3);

    public abstract char get(long... jArr);

    public abstract CharIndexer get(long j, long j2, char[] cArr, int i, int i2);

    public abstract CharIndexer get(long j, char[] cArr, int i, int i2);

    public abstract CharIndexer get(long[] jArr, char[] cArr, int i, int i2);

    public abstract CharIndexer put(long j, char c);

    public abstract CharIndexer put(long j, long j2, char c);

    public abstract CharIndexer put(long j, long j2, long j3, char c);

    public abstract CharIndexer put(long j, long j2, char[] cArr, int i, int i2);

    public abstract CharIndexer put(long j, char[] cArr, int i, int i2);

    public abstract CharIndexer put(long[] jArr, char c);

    public abstract CharIndexer put(long[] jArr, char[] cArr, int i, int i2);

    protected CharIndexer(long[] sizes, long[] strides) {
        super(sizes, strides);
    }

    public static CharIndexer create(char[] array) {
        return new CharArrayIndexer(array);
    }

    public static CharIndexer create(CharBuffer buffer) {
        return new CharBufferIndexer(buffer);
    }

    public static CharIndexer create(CharPointer pointer) {
        return create(pointer, new long[]{pointer.limit() - pointer.position()}, ONE_STRIDE);
    }

    public static CharIndexer create(char[] array, long[] sizes, long[] strides) {
        return new CharArrayIndexer(array, sizes, strides);
    }

    public static CharIndexer create(CharBuffer buffer, long[] sizes, long[] strides) {
        return new CharBufferIndexer(buffer, sizes, strides);
    }

    public static CharIndexer create(CharPointer pointer, long[] sizes, long[] strides) {
        return create(pointer, sizes, strides, true);
    }

    public static CharIndexer create(CharPointer pointer, long[] sizes, long[] strides, boolean direct) {
        if (!direct) {
            long position = pointer.position();
            char[] array = new char[((int) Math.min(pointer.limit() - position, TTL.MAX_VALUE))];
            pointer.get(array);
            final CharPointer charPointer = pointer;
            final long j = position;
            return new CharArrayIndexer(array, sizes, strides) {
                public void release() {
                    charPointer.position(j).put(this.array);
                    super.release();
                }
            };
        } else if (Raw.getInstance() != null) {
            return new CharRawIndexer(pointer, sizes, strides);
        } else {
            return new CharBufferIndexer(pointer.asBuffer(), sizes, strides);
        }
    }

    public CharIndexer get(long i, char[] c) {
        return get(i, c, 0, c.length);
    }

    public CharIndexer get(long i, long j, char[] c) {
        return get(i, j, c, 0, c.length);
    }

    public CharIndexer get(long[] indices, char[] c) {
        return get(indices, c, 0, c.length);
    }

    public CharIndexer put(long i, char... c) {
        return put(i, c, 0, c.length);
    }

    public CharIndexer put(long i, long j, char... c) {
        return put(i, j, c, 0, c.length);
    }

    public CharIndexer put(long[] indices, char... c) {
        return put(indices, c, 0, c.length);
    }

    public double getDouble(long... indices) {
        return (double) get(indices);
    }

    public CharIndexer putDouble(long[] indices, double c) {
        return put(indices, (char) ((int) c));
    }
}
