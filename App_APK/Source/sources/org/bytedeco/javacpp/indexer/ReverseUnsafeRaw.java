package org.bytedeco.javacpp.indexer;

class ReverseUnsafeRaw extends UnsafeRaw {
    ReverseUnsafeRaw() {
    }

    /* access modifiers changed from: package-private */
    public short getShort(long address) {
        return Short.reverseBytes(super.getShort(address));
    }

    /* access modifiers changed from: package-private */
    public void putShort(long address, short s) {
        super.putShort(address, Short.reverseBytes(s));
    }

    /* access modifiers changed from: package-private */
    public int getInt(long address) {
        return Integer.reverseBytes(super.getInt(address));
    }

    /* access modifiers changed from: package-private */
    public void putInt(long address, int i) {
        super.putInt(address, Integer.reverseBytes(i));
    }

    /* access modifiers changed from: package-private */
    public long getLong(long address) {
        return Long.reverseBytes(super.getLong(address));
    }

    /* access modifiers changed from: package-private */
    public void putLong(long address, long l) {
        super.putLong(address, Long.reverseBytes(l));
    }

    /* access modifiers changed from: package-private */
    public float getFloat(long address) {
        return Float.intBitsToFloat(Integer.reverseBytes(super.getInt(address)));
    }

    /* access modifiers changed from: package-private */
    public void putFloat(long address, float f) {
        super.putFloat(address, (float) Integer.reverseBytes(Float.floatToRawIntBits(f)));
    }

    /* access modifiers changed from: package-private */
    public double getDouble(long address) {
        return Double.longBitsToDouble(Long.reverseBytes(super.getLong(address)));
    }

    /* access modifiers changed from: package-private */
    public void putDouble(long address, double d) {
        super.putDouble(address, (double) Long.reverseBytes(Double.doubleToRawLongBits(d)));
    }

    /* access modifiers changed from: package-private */
    public char getChar(long address) {
        return Character.reverseBytes(super.getChar(address));
    }

    /* access modifiers changed from: package-private */
    public void putChar(long address, char c) {
        super.putChar(address, Character.reverseBytes(c));
    }

    /* access modifiers changed from: package-private */
    public short getShort(byte[] array, long offset) {
        return Short.reverseBytes(super.getShort(array, offset));
    }

    /* access modifiers changed from: package-private */
    public void putShort(byte[] array, long offset, short s) {
        super.putShort(array, offset, Short.reverseBytes(s));
    }

    /* access modifiers changed from: package-private */
    public int getInt(byte[] array, long offset) {
        return Integer.reverseBytes(super.getInt(array, offset));
    }

    /* access modifiers changed from: package-private */
    public void putInt(byte[] array, long offset, int i) {
        super.putInt(array, offset, Integer.reverseBytes(i));
    }

    /* access modifiers changed from: package-private */
    public long getLong(byte[] array, long offset) {
        return Long.reverseBytes(super.getLong(array, offset));
    }

    /* access modifiers changed from: package-private */
    public void putLong(byte[] array, long offset, long l) {
        super.putLong(array, offset, Long.reverseBytes(l));
    }

    /* access modifiers changed from: package-private */
    public float getFloat(byte[] array, long offset) {
        return Float.intBitsToFloat(Integer.reverseBytes(super.getInt(array, offset)));
    }

    /* access modifiers changed from: package-private */
    public void putFloat(byte[] array, long offset, float f) {
        super.putFloat(array, offset, (float) Integer.reverseBytes(Float.floatToRawIntBits(f)));
    }

    /* access modifiers changed from: package-private */
    public double getDouble(byte[] array, long offset) {
        return Double.longBitsToDouble(Long.reverseBytes(super.getLong(array, offset)));
    }

    /* access modifiers changed from: package-private */
    public void putDouble(byte[] array, long offset, double d) {
        super.putDouble(array, offset, (double) Long.reverseBytes(Double.doubleToRawLongBits(d)));
    }

    /* access modifiers changed from: package-private */
    public char getChar(byte[] array, long offset) {
        return Character.reverseBytes(super.getChar(array, offset));
    }

    /* access modifiers changed from: package-private */
    public void putChar(byte[] array, long offset, char c) {
        super.putChar(array, offset, Character.reverseBytes(c));
    }
}
