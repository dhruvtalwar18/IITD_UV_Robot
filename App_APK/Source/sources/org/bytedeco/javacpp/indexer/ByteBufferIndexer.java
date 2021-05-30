package org.bytedeco.javacpp.indexer;

import java.nio.Buffer;
import java.nio.ByteBuffer;

public class ByteBufferIndexer extends ByteIndexer {
    protected ByteBuffer buffer;

    public ByteBufferIndexer(ByteBuffer buffer2) {
        this(buffer2, new long[]{(long) buffer2.limit()}, ONE_STRIDE);
    }

    public ByteBufferIndexer(ByteBuffer buffer2, long[] sizes, long[] strides) {
        super(sizes, strides);
        this.buffer = buffer2;
    }

    public Buffer buffer() {
        return this.buffer;
    }

    public byte get(long i) {
        return this.buffer.get((int) i);
    }

    public ByteIndexer get(long i, byte[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            b[offset + n] = this.buffer.get((((int) i) * ((int) this.strides[0])) + n);
        }
        return this;
    }

    public byte get(long i, long j) {
        return this.buffer.get((((int) i) * ((int) this.strides[0])) + ((int) j));
    }

    public ByteIndexer get(long i, long j, byte[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            b[offset + n] = this.buffer.get((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n);
        }
        long j2 = i;
        long j3 = j;
        return this;
    }

    public byte get(long i, long j, long k) {
        return this.buffer.get((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k));
    }

    public byte get(long... indices) {
        return this.buffer.get((int) index(indices));
    }

    public ByteIndexer get(long[] indices, byte[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            b[offset + n] = this.buffer.get(((int) index(indices)) + n);
        }
        return this;
    }

    public ByteIndexer put(long i, byte b) {
        this.buffer.put((int) i, b);
        return this;
    }

    public ByteIndexer put(long i, byte[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put((((int) i) * ((int) this.strides[0])) + n, b[offset + n]);
        }
        return this;
    }

    public ByteIndexer put(long i, long j, byte b) {
        this.buffer.put((((int) i) * ((int) this.strides[0])) + ((int) j), b);
        return this;
    }

    public ByteIndexer put(long i, long j, byte[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + n, b[offset + n]);
        }
        return this;
    }

    public ByteIndexer put(long i, long j, long k, byte b) {
        this.buffer.put((((int) i) * ((int) this.strides[0])) + (((int) j) * ((int) this.strides[1])) + ((int) k), b);
        return this;
    }

    public ByteIndexer put(long[] indices, byte b) {
        this.buffer.put((int) index(indices), b);
        return this;
    }

    public ByteIndexer put(long[] indices, byte[] b, int offset, int length) {
        for (int n = 0; n < length; n++) {
            this.buffer.put(((int) index(indices)) + n, b[offset + n]);
        }
        return this;
    }

    public short getShort(long i) {
        return this.buffer.getShort((int) i);
    }

    public ByteIndexer putShort(long i, short s) {
        this.buffer.putShort((int) i, s);
        return this;
    }

    public int getInt(long i) {
        return this.buffer.getInt((int) i);
    }

    public ByteIndexer putInt(long i, int j) {
        this.buffer.putInt((int) i, j);
        return this;
    }

    public long getLong(long i) {
        return this.buffer.getLong((int) i);
    }

    public ByteIndexer putLong(long i, long j) {
        this.buffer.putLong((int) i, j);
        return this;
    }

    public float getFloat(long i) {
        return this.buffer.getFloat((int) i);
    }

    public ByteIndexer putFloat(long i, float f) {
        this.buffer.putFloat((int) i, f);
        return this;
    }

    public double getDouble(long i) {
        return this.buffer.getDouble((int) i);
    }

    public ByteIndexer putDouble(long i, double d) {
        this.buffer.putDouble((int) i, d);
        return this;
    }

    public char getChar(long i) {
        return this.buffer.getChar((int) i);
    }

    public ByteIndexer putChar(long i, char c) {
        this.buffer.putChar((int) i, c);
        return this;
    }

    public void release() {
        this.buffer = null;
    }
}
