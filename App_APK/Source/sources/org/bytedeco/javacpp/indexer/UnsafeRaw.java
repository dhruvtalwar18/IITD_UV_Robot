package org.bytedeco.javacpp.indexer;

import java.lang.reflect.Field;
import sun.misc.Unsafe;

class UnsafeRaw extends Raw {
    protected static final Unsafe UNSAFE;
    protected static final long arrayOffset;

    UnsafeRaw() {
    }

    static {
        Unsafe o;
        long offset;
        try {
            Class c = Class.forName("sun.misc.Unsafe");
            Field f = c.getDeclaredField("theUnsafe");
            c.getDeclaredMethod("getByte", new Class[]{Long.TYPE});
            c.getDeclaredMethod("getShort", new Class[]{Long.TYPE});
            c.getDeclaredMethod("getInt", new Class[]{Long.TYPE});
            c.getDeclaredMethod("getLong", new Class[]{Long.TYPE});
            c.getDeclaredMethod("getFloat", new Class[]{Long.TYPE});
            c.getDeclaredMethod("getDouble", new Class[]{Long.TYPE});
            c.getDeclaredMethod("getChar", new Class[]{Long.TYPE});
            c.getDeclaredMethod("arrayBaseOffset", new Class[]{Class.class});
            f.setAccessible(true);
            o = (Unsafe) f.get((Object) null);
            offset = (long) o.arrayBaseOffset(byte[].class);
        } catch (ClassNotFoundException | IllegalAccessException | IllegalArgumentException | NoSuchFieldException | NoSuchMethodException | SecurityException e) {
            o = null;
            offset = 0;
        }
        UNSAFE = o;
        arrayOffset = offset;
    }

    static boolean isAvailable() {
        return UNSAFE != null;
    }

    /* access modifiers changed from: package-private */
    public byte getByte(long address) {
        return UNSAFE.getByte(address);
    }

    /* access modifiers changed from: package-private */
    public void putByte(long address, byte b) {
        UNSAFE.putByte(address, b);
    }

    /* access modifiers changed from: package-private */
    public short getShort(long address) {
        return UNSAFE.getShort(address);
    }

    /* access modifiers changed from: package-private */
    public void putShort(long address, short s) {
        UNSAFE.putShort(address, s);
    }

    /* access modifiers changed from: package-private */
    public int getInt(long address) {
        return UNSAFE.getInt(address);
    }

    /* access modifiers changed from: package-private */
    public void putInt(long address, int i) {
        UNSAFE.putInt(address, i);
    }

    /* access modifiers changed from: package-private */
    public long getLong(long address) {
        return UNSAFE.getLong(address);
    }

    /* access modifiers changed from: package-private */
    public void putLong(long address, long l) {
        UNSAFE.putLong(address, l);
    }

    /* access modifiers changed from: package-private */
    public float getFloat(long address) {
        return UNSAFE.getFloat(address);
    }

    /* access modifiers changed from: package-private */
    public void putFloat(long address, float f) {
        UNSAFE.putFloat(address, f);
    }

    /* access modifiers changed from: package-private */
    public double getDouble(long address) {
        return UNSAFE.getDouble(address);
    }

    /* access modifiers changed from: package-private */
    public void putDouble(long address, double d) {
        UNSAFE.putDouble(address, d);
    }

    /* access modifiers changed from: package-private */
    public char getChar(long address) {
        return UNSAFE.getChar(address);
    }

    /* access modifiers changed from: package-private */
    public void putChar(long address, char c) {
        UNSAFE.putChar(address, c);
    }

    /* access modifiers changed from: package-private */
    public boolean getBoolean(long address) {
        return UNSAFE.getByte(address) != 0;
    }

    /* access modifiers changed from: package-private */
    public void putBoolean(long address, boolean b) {
        UNSAFE.putByte(address, b);
    }

    /* access modifiers changed from: package-private */
    public byte getByte(byte[] array, long offset) {
        return UNSAFE.getByte(array, arrayOffset + offset);
    }

    /* access modifiers changed from: package-private */
    public void putByte(byte[] array, long offset, byte b) {
        UNSAFE.putByte(array, arrayOffset + offset, b);
    }

    /* access modifiers changed from: package-private */
    public short getShort(byte[] array, long offset) {
        return UNSAFE.getShort(array, arrayOffset + offset);
    }

    /* access modifiers changed from: package-private */
    public void putShort(byte[] array, long offset, short s) {
        UNSAFE.putShort(array, arrayOffset + offset, s);
    }

    /* access modifiers changed from: package-private */
    public int getInt(byte[] array, long offset) {
        return UNSAFE.getInt(array, arrayOffset + offset);
    }

    /* access modifiers changed from: package-private */
    public void putInt(byte[] array, long offset, int i) {
        UNSAFE.putInt(array, arrayOffset + offset, i);
    }

    /* access modifiers changed from: package-private */
    public long getLong(byte[] array, long offset) {
        return UNSAFE.getLong(array, arrayOffset + offset);
    }

    /* access modifiers changed from: package-private */
    public void putLong(byte[] array, long offset, long l) {
        UNSAFE.putLong(array, arrayOffset + offset, l);
    }

    /* access modifiers changed from: package-private */
    public float getFloat(byte[] array, long offset) {
        return UNSAFE.getFloat(array, arrayOffset + offset);
    }

    /* access modifiers changed from: package-private */
    public void putFloat(byte[] array, long offset, float f) {
        UNSAFE.putFloat(array, arrayOffset + offset, f);
    }

    /* access modifiers changed from: package-private */
    public double getDouble(byte[] array, long offset) {
        return UNSAFE.getDouble(array, arrayOffset + offset);
    }

    /* access modifiers changed from: package-private */
    public void putDouble(byte[] array, long offset, double d) {
        UNSAFE.putDouble(array, arrayOffset + offset, d);
    }

    /* access modifiers changed from: package-private */
    public char getChar(byte[] array, long offset) {
        return UNSAFE.getChar(array, arrayOffset + offset);
    }

    /* access modifiers changed from: package-private */
    public void putChar(byte[] array, long offset, char c) {
        UNSAFE.putChar(array, arrayOffset + offset, c);
    }

    /* access modifiers changed from: package-private */
    public boolean getBoolean(byte[] array, long offset) {
        return UNSAFE.getBoolean(array, arrayOffset + offset);
    }

    /* access modifiers changed from: package-private */
    public void putBoolean(byte[] array, long offset, boolean b) {
        UNSAFE.putBoolean(array, arrayOffset + offset, b);
    }
}
