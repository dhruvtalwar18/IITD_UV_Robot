package org.jboss.netty.buffer;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.GatheringByteChannel;
import java.nio.channels.ScatteringByteChannel;

public class SlicedChannelBuffer extends AbstractChannelBuffer implements WrappedChannelBuffer {
    private final int adjustment;
    private final ChannelBuffer buffer;
    private final int length;

    public SlicedChannelBuffer(ChannelBuffer buffer2, int index, int length2) {
        if (index < 0 || index > buffer2.capacity()) {
            throw new IndexOutOfBoundsException("Invalid index of " + index + ", maximum is " + buffer2.capacity());
        } else if (index + length2 <= buffer2.capacity()) {
            this.buffer = buffer2;
            this.adjustment = index;
            this.length = length2;
            writerIndex(length2);
        } else {
            throw new IndexOutOfBoundsException("Invalid combined index of " + (index + length2) + ", maximum is " + buffer2.capacity());
        }
    }

    public ChannelBuffer unwrap() {
        return this.buffer;
    }

    public ChannelBufferFactory factory() {
        return this.buffer.factory();
    }

    public ByteOrder order() {
        return this.buffer.order();
    }

    public boolean isDirect() {
        return this.buffer.isDirect();
    }

    public int capacity() {
        return this.length;
    }

    public boolean hasArray() {
        return this.buffer.hasArray();
    }

    public byte[] array() {
        return this.buffer.array();
    }

    public int arrayOffset() {
        return this.buffer.arrayOffset() + this.adjustment;
    }

    public byte getByte(int index) {
        checkIndex(index);
        return this.buffer.getByte(this.adjustment + index);
    }

    public short getShort(int index) {
        checkIndex(index, 2);
        return this.buffer.getShort(this.adjustment + index);
    }

    public int getUnsignedMedium(int index) {
        checkIndex(index, 3);
        return this.buffer.getUnsignedMedium(this.adjustment + index);
    }

    public int getInt(int index) {
        checkIndex(index, 4);
        return this.buffer.getInt(this.adjustment + index);
    }

    public long getLong(int index) {
        checkIndex(index, 8);
        return this.buffer.getLong(this.adjustment + index);
    }

    public ChannelBuffer duplicate() {
        ChannelBuffer duplicate = new SlicedChannelBuffer(this.buffer, this.adjustment, this.length);
        duplicate.setIndex(readerIndex(), writerIndex());
        return duplicate;
    }

    public ChannelBuffer copy(int index, int length2) {
        checkIndex(index, length2);
        return this.buffer.copy(this.adjustment + index, length2);
    }

    public ChannelBuffer slice(int index, int length2) {
        checkIndex(index, length2);
        if (length2 == 0) {
            return ChannelBuffers.EMPTY_BUFFER;
        }
        return new SlicedChannelBuffer(this.buffer, this.adjustment + index, length2);
    }

    public void getBytes(int index, ChannelBuffer dst, int dstIndex, int length2) {
        checkIndex(index, length2);
        this.buffer.getBytes(this.adjustment + index, dst, dstIndex, length2);
    }

    public void getBytes(int index, byte[] dst, int dstIndex, int length2) {
        checkIndex(index, length2);
        this.buffer.getBytes(this.adjustment + index, dst, dstIndex, length2);
    }

    public void getBytes(int index, ByteBuffer dst) {
        checkIndex(index, dst.remaining());
        this.buffer.getBytes(this.adjustment + index, dst);
    }

    public void setByte(int index, int value) {
        checkIndex(index);
        this.buffer.setByte(this.adjustment + index, value);
    }

    public void setShort(int index, int value) {
        checkIndex(index, 2);
        this.buffer.setShort(this.adjustment + index, value);
    }

    public void setMedium(int index, int value) {
        checkIndex(index, 3);
        this.buffer.setMedium(this.adjustment + index, value);
    }

    public void setInt(int index, int value) {
        checkIndex(index, 4);
        this.buffer.setInt(this.adjustment + index, value);
    }

    public void setLong(int index, long value) {
        checkIndex(index, 8);
        this.buffer.setLong(this.adjustment + index, value);
    }

    public void setBytes(int index, byte[] src, int srcIndex, int length2) {
        checkIndex(index, length2);
        this.buffer.setBytes(this.adjustment + index, src, srcIndex, length2);
    }

    public void setBytes(int index, ChannelBuffer src, int srcIndex, int length2) {
        checkIndex(index, length2);
        this.buffer.setBytes(this.adjustment + index, src, srcIndex, length2);
    }

    public void setBytes(int index, ByteBuffer src) {
        checkIndex(index, src.remaining());
        this.buffer.setBytes(this.adjustment + index, src);
    }

    public void getBytes(int index, OutputStream out, int length2) throws IOException {
        checkIndex(index, length2);
        this.buffer.getBytes(this.adjustment + index, out, length2);
    }

    public int getBytes(int index, GatheringByteChannel out, int length2) throws IOException {
        checkIndex(index, length2);
        return this.buffer.getBytes(this.adjustment + index, out, length2);
    }

    public int setBytes(int index, InputStream in, int length2) throws IOException {
        checkIndex(index, length2);
        return this.buffer.setBytes(this.adjustment + index, in, length2);
    }

    public int setBytes(int index, ScatteringByteChannel in, int length2) throws IOException {
        checkIndex(index, length2);
        return this.buffer.setBytes(this.adjustment + index, in, length2);
    }

    public ByteBuffer toByteBuffer(int index, int length2) {
        checkIndex(index, length2);
        return this.buffer.toByteBuffer(this.adjustment + index, length2);
    }

    private void checkIndex(int index) {
        if (index < 0 || index >= capacity()) {
            throw new IndexOutOfBoundsException("Invalid index: " + index + ", maximum is " + capacity());
        }
    }

    private void checkIndex(int startIndex, int length2) {
        if (length2 < 0) {
            throw new IllegalArgumentException("length is negative: " + length2);
        } else if (startIndex < 0) {
            throw new IndexOutOfBoundsException("startIndex cannot be negative");
        } else if (startIndex + length2 > capacity()) {
            throw new IndexOutOfBoundsException("Index too big - Bytes needed: " + (startIndex + length2) + ", maximum is " + capacity());
        }
    }
}
