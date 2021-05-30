package org.bytedeco.javacpp.indexer;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class ByteArrayIndexer extends ByteIndexer {
    protected static final Raw RAW = Raw.getInstance();
    protected byte[] array;
    protected ByteBuffer buffer;

    public /* bridge */ /* synthetic */ Indexer putDouble(long[] jArr, double d) {
        return super.putDouble(jArr, d);
    }

    public ByteArrayIndexer(byte[] array2) {
        this(array2, new long[]{(long) array2.length}, ONE_STRIDE);
    }

    public ByteArrayIndexer(byte[] array2, long[] sizes, long[] strides) {
        super(sizes, strides);
        this.array = array2;
    }

    public byte[] array() {
        return this.array;
    }

    public byte get(long i) {
        return this.array[(int) i];
    }

    public ByteIndexer get(long i, byte[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            b[offset + n] = this.array[(((int) i) * ((int) this.strides[0])) + n];
        }
        return this;
    }

    public byte get(long i, long j) {
        return this.array[(((int) i) * ((int) this.strides[0])) + ((int) j)];
    }

    public ByteIndexer get(long i, long j, byte[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            b[offset + n] = this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n];
        }
        long j2 = i;
        long j3 = j;
        return this;
    }

    public byte get(long i, long j, long k) {
        return this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k)];
    }

    public byte get(long... indices) {
        return this.array[(int) index(indices)];
    }

    public ByteIndexer get(long[] indices, byte[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            b[offset + n] = this.array[((int) index(indices)) + n];
        }
        return this;
    }

    public ByteIndexer put(long i, byte b) {
        this.array[(int) i] = b;
        return this;
    }

    public ByteIndexer put(long i, byte[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.array[(((int) i) * ((int) this.strides[0])) + n] = b[offset + n];
        }
        return this;
    }

    public ByteIndexer put(long i, long j, byte b) {
        this.array[(((int) i) * ((int) this.strides[0])) + ((int) j)] = b;
        return this;
    }

    public ByteIndexer put(long i, long j, byte[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n] = b[offset + n];
        }
        return this;
    }

    public ByteIndexer put(long i, long j, long k, byte b) {
        this.array[(((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k)] = b;
        return this;
    }

    public ByteIndexer put(long[] indices, byte b) {
        this.array[(int) index(indices)] = b;
        return this;
    }

    public ByteIndexer put(long[] indices, byte[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.array[((int) index(indices)) + n] = b[offset + n];
        }
        return this;
    }

    /* access modifiers changed from: package-private */
    public ByteBuffer getBuffer() {
        if (this.buffer == null) {
            this.buffer = ByteBuffer.wrap(this.array).order(ByteOrder.nativeOrder());
        }
        return this.buffer;
    }

    public short getShort(long i) {
        if (RAW != null) {
            return RAW.getShort(this.array, checkIndex(i, (long) (this.array.length - 1)));
        }
        return getBuffer().getShort((int) i);
    }

    public ByteIndexer putShort(long i, short s) {
        if (RAW != null) {
            RAW.putShort(this.array, checkIndex(i, (long) (this.array.length - 1)), s);
        } else {
            getBuffer().putShort((int) i, s);
        }
        return this;
    }

    public int getInt(long i) {
        if (RAW != null) {
            return RAW.getInt(this.array, checkIndex(i, (long) (this.array.length - 3)));
        }
        return getBuffer().getInt((int) i);
    }

    public ByteIndexer putInt(long i, int j) {
        if (RAW != null) {
            RAW.putInt(this.array, checkIndex(i, (long) (this.array.length - 3)), j);
        } else {
            getBuffer().putInt((int) i, j);
        }
        return this;
    }

    public long getLong(long i) {
        if (RAW != null) {
            return RAW.getLong(this.array, checkIndex(i, (long) (this.array.length - 7)));
        }
        return getBuffer().getLong((int) i);
    }

    public ByteIndexer putLong(long i, long j) {
        if (RAW != null) {
            RAW.putLong(this.array, checkIndex(i, (long) (this.array.length - 7)), j);
        } else {
            getBuffer().putLong((int) i, j);
        }
        return this;
    }

    public float getFloat(long i) {
        if (RAW != null) {
            return RAW.getFloat(this.array, checkIndex(i, (long) (this.array.length - 3)));
        }
        return getBuffer().getFloat((int) i);
    }

    public ByteIndexer putFloat(long i, float f) {
        if (RAW != null) {
            RAW.putFloat(this.array, checkIndex(i, (long) (this.array.length - 3)), f);
        } else {
            getBuffer().putFloat((int) i, f);
        }
        return this;
    }

    public double getDouble(long i) {
        if (RAW != null) {
            return RAW.getDouble(this.array, checkIndex(i, (long) (this.array.length - 7)));
        }
        return getBuffer().getDouble((int) i);
    }

    public ByteIndexer putDouble(long i, double d) {
        if (RAW != null) {
            RAW.putDouble(this.array, checkIndex(i, (long) (this.array.length - 7)), d);
        } else {
            getBuffer().putDouble((int) i, d);
        }
        return this;
    }

    public char getChar(long i) {
        if (RAW != null) {
            return RAW.getChar(this.array, checkIndex(i, (long) (this.array.length - 1)));
        }
        return getBuffer().getChar((int) i);
    }

    public ByteIndexer putChar(long i, char c) {
        if (RAW != null) {
            RAW.putChar(this.array, checkIndex(i, (long) (this.array.length - 1)), c);
        } else {
            getBuffer().putChar((int) i, c);
        }
        return this;
    }

    public void release() {
        this.array = null;
    }
}
