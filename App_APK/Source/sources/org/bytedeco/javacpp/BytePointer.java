package org.bytedeco.javacpp;

import java.io.UnsupportedEncodingException;
import java.nio.Buffer;
import java.nio.ByteBuffer;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.ValueGetter;
import org.bytedeco.javacpp.annotation.ValueSetter;
import org.xbill.DNS.TTL;

public class BytePointer extends Pointer {
    private native void allocateArray(long j);

    @Cast({"char*"})
    public static native BytePointer strcat(@Cast({"char*"}) BytePointer bytePointer, @Cast({"char*"}) BytePointer bytePointer2);

    @Cast({"char*"})
    public static native BytePointer strchr(@Cast({"char*"}) BytePointer bytePointer, int i);

    public static native int strcmp(@Cast({"char*"}) BytePointer bytePointer, @Cast({"char*"}) BytePointer bytePointer2);

    public static native int strcoll(@Cast({"char*"}) BytePointer bytePointer, @Cast({"char*"}) BytePointer bytePointer2);

    @Cast({"char*"})
    public static native BytePointer strcpy(@Cast({"char*"}) BytePointer bytePointer, @Cast({"char*"}) BytePointer bytePointer2);

    @Cast({"size_t"})
    public static native long strcspn(@Cast({"char*"}) BytePointer bytePointer, @Cast({"char*"}) BytePointer bytePointer2);

    @Cast({"char*"})
    public static native BytePointer strerror(int i);

    @Cast({"size_t"})
    public static native long strlen(@Cast({"char*"}) BytePointer bytePointer);

    @Cast({"char*"})
    public static native BytePointer strncat(@Cast({"char*"}) BytePointer bytePointer, @Cast({"char*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j);

    public static native int strncmp(@Cast({"char*"}) BytePointer bytePointer, @Cast({"char*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j);

    @Cast({"char*"})
    public static native BytePointer strncpy(@Cast({"char*"}) BytePointer bytePointer, @Cast({"char*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j);

    @Cast({"char*"})
    public static native BytePointer strpbrk(@Cast({"char*"}) BytePointer bytePointer, @Cast({"char*"}) BytePointer bytePointer2);

    @Cast({"char*"})
    public static native BytePointer strrchr(@Cast({"char*"}) BytePointer bytePointer, int i);

    @Cast({"size_t"})
    public static native long strspn(@Cast({"char*"}) BytePointer bytePointer, @Cast({"char*"}) BytePointer bytePointer2);

    @Cast({"char*"})
    public static native BytePointer strstr(@Cast({"char*"}) BytePointer bytePointer, @Cast({"char*"}) BytePointer bytePointer2);

    @Cast({"char*"})
    public static native BytePointer strtok(@Cast({"char*"}) BytePointer bytePointer, @Cast({"char*"}) BytePointer bytePointer2);

    @Cast({"size_t"})
    public static native long strxfrm(@Cast({"char*"}) BytePointer bytePointer, @Cast({"char*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j);

    public native byte get(long j);

    public native BytePointer get(byte[] bArr, int i, int i2);

    @Cast({"bool"})
    @ValueGetter
    public native boolean getBool(long j);

    @Cast({"short"})
    @ValueGetter
    public native char getChar(long j);

    @Cast({"double"})
    @ValueGetter
    public native double getDouble(long j);

    @Cast({"float"})
    @ValueGetter
    public native float getFloat(long j);

    @Cast({"int"})
    @ValueGetter
    public native int getInt(long j);

    @Cast({"long long"})
    @ValueGetter
    public native long getLong(long j);

    @Cast({"void*"})
    @ValueGetter
    public native Pointer getPointer(long j);

    @Cast({"short"})
    @ValueGetter
    public native short getShort(long j);

    public native BytePointer put(long j, byte b);

    public native BytePointer put(byte[] bArr, int i, int i2);

    @ValueSetter
    @Cast({"bool"})
    public native BytePointer putBool(long j, boolean z);

    @ValueSetter
    @Cast({"short"})
    public native BytePointer putChar(long j, char c);

    @ValueSetter
    @Cast({"double"})
    public native BytePointer putDouble(long j, double d);

    @ValueSetter
    @Cast({"float"})
    public native BytePointer putFloat(long j, float f);

    @ValueSetter
    @Cast({"int"})
    public native BytePointer putInt(long j, int i);

    @ValueSetter
    @Cast({"long long"})
    public native BytePointer putLong(long j, long j2);

    @ValueSetter
    @Cast({"void*"})
    public native BytePointer putPointer(long j, Pointer pointer);

    @ValueSetter
    @Cast({"short"})
    public native BytePointer putShort(long j, short s);

    public BytePointer(String s, String charsetName) throws UnsupportedEncodingException {
        this((long) (s.getBytes(charsetName).length + 1));
        putString(s, charsetName);
    }

    public BytePointer(String s) {
        this((long) (s.getBytes().length + 1));
        putString(s);
    }

    public BytePointer(byte... array) {
        this((long) array.length);
        put(array);
    }

    public BytePointer(ByteBuffer buffer) {
        super((Buffer) buffer);
        if (buffer != null && !buffer.isDirect() && buffer.hasArray()) {
            byte[] array = buffer.array();
            allocateArray((long) (array.length - buffer.arrayOffset()));
            put(array, buffer.arrayOffset(), array.length - buffer.arrayOffset());
            position((long) buffer.position());
            limit((long) buffer.limit());
        }
    }

    public BytePointer(long size) {
        try {
            allocateArray(size);
            if (size <= 0) {
                return;
            }
            if (this.address == 0) {
                throw new OutOfMemoryError("Native allocator returned address == 0");
            }
        } catch (UnsatisfiedLinkError e) {
            throw new RuntimeException("No native JavaCPP library in memory. (Has Loader.load() been called?)", e);
        } catch (OutOfMemoryError e2) {
            OutOfMemoryError e22 = new OutOfMemoryError("Cannot allocate new BytePointer(" + size + "): totalBytes = " + formatBytes(totalBytes()) + ", physicalBytes = " + formatBytes(physicalBytes()));
            e22.initCause(e2);
            throw e22;
        }
    }

    public BytePointer() {
    }

    public BytePointer(Pointer p) {
        super(p);
    }

    public BytePointer position(long position) {
        return (BytePointer) super.position(position);
    }

    public BytePointer limit(long limit) {
        return (BytePointer) super.limit(limit);
    }

    public BytePointer capacity(long capacity) {
        return (BytePointer) super.capacity(capacity);
    }

    public byte[] getStringBytes() {
        long size = this.limit - this.position;
        if (size <= 0) {
            size = strlen(this);
        }
        byte[] array = new byte[((int) Math.min(size, TTL.MAX_VALUE))];
        get(array);
        return array;
    }

    public String getString(String charsetName) throws UnsupportedEncodingException {
        return new String(getStringBytes(), charsetName);
    }

    public String getString() {
        return new String(getStringBytes());
    }

    public BytePointer putString(String s, String charsetName) throws UnsupportedEncodingException {
        byte[] bytes = s.getBytes(charsetName);
        return put(bytes).put((long) bytes.length, (byte) 0).limit((long) bytes.length);
    }

    public BytePointer putString(String s) {
        byte[] bytes = s.getBytes();
        return put(bytes).put((long) bytes.length, (byte) 0).limit((long) bytes.length);
    }

    public byte get() {
        return get(0);
    }

    public BytePointer put(byte b) {
        return put(0, b);
    }

    public BytePointer get(byte[] array) {
        return get(array, 0, array.length);
    }

    public BytePointer put(byte... array) {
        return put(array, 0, array.length);
    }

    public final ByteBuffer asBuffer() {
        return asByteBuffer();
    }

    public short getShort() {
        return getShort(0);
    }

    public BytePointer putShort(short s) {
        return putShort(0, s);
    }

    public int getInt() {
        return getInt(0);
    }

    public BytePointer putInt(int j) {
        return putInt(0, j);
    }

    public long getLong() {
        return getLong(0);
    }

    public BytePointer putLong(long j) {
        return putLong(0, j);
    }

    public float getFloat() {
        return getFloat(0);
    }

    public BytePointer putFloat(float f) {
        return putFloat(0, f);
    }

    public double getDouble() {
        return getDouble(0);
    }

    public BytePointer putDouble(double d) {
        return putDouble(0, d);
    }

    public boolean getBool() {
        return getBool(0);
    }

    public BytePointer putBool(boolean b) {
        return putBool(0, b);
    }

    public char getChar() {
        return getChar(0);
    }

    public BytePointer putChar(char c) {
        return putChar(0, c);
    }

    public Pointer getPointer() {
        return getPointer(0);
    }

    public BytePointer putPointer(Pointer p) {
        return putPointer(0, p);
    }
}
