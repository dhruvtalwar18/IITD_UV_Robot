package org.jboss.netty.buffer;

import com.google.common.base.Ascii;
import java.nio.ByteOrder;
import sensor_msgs.NavSatStatus;

public class LittleEndianHeapChannelBuffer extends HeapChannelBuffer {
    public LittleEndianHeapChannelBuffer(int length) {
        super(length);
    }

    public LittleEndianHeapChannelBuffer(byte[] array) {
        super(array);
    }

    private LittleEndianHeapChannelBuffer(byte[] array, int readerIndex, int writerIndex) {
        super(array, readerIndex, writerIndex);
    }

    public ChannelBufferFactory factory() {
        return HeapChannelBufferFactory.getInstance(ByteOrder.LITTLE_ENDIAN);
    }

    public ByteOrder order() {
        return ByteOrder.LITTLE_ENDIAN;
    }

    public short getShort(int index) {
        return (short) ((this.array[index] & NavSatStatus.STATUS_NO_FIX) | (this.array[index + 1] << 8));
    }

    public int getUnsignedMedium(int index) {
        return ((this.array[index] & NavSatStatus.STATUS_NO_FIX) << 0) | ((this.array[index + 1] & NavSatStatus.STATUS_NO_FIX) << 8) | ((this.array[index + 2] & NavSatStatus.STATUS_NO_FIX) << 16);
    }

    public int getInt(int index) {
        return ((this.array[index] & NavSatStatus.STATUS_NO_FIX) << 0) | ((this.array[index + 1] & NavSatStatus.STATUS_NO_FIX) << 8) | ((this.array[index + 2] & NavSatStatus.STATUS_NO_FIX) << 16) | ((this.array[index + 3] & NavSatStatus.STATUS_NO_FIX) << Ascii.CAN);
    }

    public long getLong(int index) {
        return ((((long) this.array[index]) & 255) << 0) | ((((long) this.array[index + 1]) & 255) << 8) | ((((long) this.array[index + 2]) & 255) << 16) | ((((long) this.array[index + 3]) & 255) << 24) | ((((long) this.array[index + 4]) & 255) << 32) | ((((long) this.array[index + 5]) & 255) << 40) | ((((long) this.array[index + 6]) & 255) << 48) | ((255 & ((long) this.array[index + 7])) << 56);
    }

    public void setShort(int index, int value) {
        this.array[index] = (byte) (value >>> 0);
        this.array[index + 1] = (byte) (value >>> 8);
    }

    public void setMedium(int index, int value) {
        this.array[index] = (byte) (value >>> 0);
        this.array[index + 1] = (byte) (value >>> 8);
        this.array[index + 2] = (byte) (value >>> 16);
    }

    public void setInt(int index, int value) {
        this.array[index] = (byte) (value >>> 0);
        this.array[index + 1] = (byte) (value >>> 8);
        this.array[index + 2] = (byte) (value >>> 16);
        this.array[index + 3] = (byte) (value >>> 24);
    }

    public void setLong(int index, long value) {
        this.array[index] = (byte) ((int) (value >>> 0));
        this.array[index + 1] = (byte) ((int) (value >>> 8));
        this.array[index + 2] = (byte) ((int) (value >>> 16));
        this.array[index + 3] = (byte) ((int) (value >>> 24));
        this.array[index + 4] = (byte) ((int) (value >>> 32));
        this.array[index + 5] = (byte) ((int) (value >>> 40));
        this.array[index + 6] = (byte) ((int) (value >>> 48));
        this.array[index + 7] = (byte) ((int) (value >>> 56));
    }

    public ChannelBuffer duplicate() {
        return new LittleEndianHeapChannelBuffer(this.array, readerIndex(), writerIndex());
    }

    public ChannelBuffer copy(int index, int length) {
        if (index < 0 || length < 0 || index + length > this.array.length) {
            throw new IndexOutOfBoundsException("Too many bytes to copy - Need " + (index + length) + ", maximum is " + this.array.length);
        }
        byte[] copiedArray = new byte[length];
        System.arraycopy(this.array, index, copiedArray, 0, length);
        return new LittleEndianHeapChannelBuffer(copiedArray);
    }
}
