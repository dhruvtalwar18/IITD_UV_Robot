package org.bytedeco.javacpp.indexer;

abstract class Raw {
    static final Raw INSTANCE;

    /* access modifiers changed from: package-private */
    public abstract boolean getBoolean(long j);

    /* access modifiers changed from: package-private */
    public abstract boolean getBoolean(byte[] bArr, long j);

    /* access modifiers changed from: package-private */
    public abstract byte getByte(long j);

    /* access modifiers changed from: package-private */
    public abstract byte getByte(byte[] bArr, long j);

    /* access modifiers changed from: package-private */
    public abstract char getChar(long j);

    /* access modifiers changed from: package-private */
    public abstract char getChar(byte[] bArr, long j);

    /* access modifiers changed from: package-private */
    public abstract double getDouble(long j);

    /* access modifiers changed from: package-private */
    public abstract double getDouble(byte[] bArr, long j);

    /* access modifiers changed from: package-private */
    public abstract float getFloat(long j);

    /* access modifiers changed from: package-private */
    public abstract float getFloat(byte[] bArr, long j);

    /* access modifiers changed from: package-private */
    public abstract int getInt(long j);

    /* access modifiers changed from: package-private */
    public abstract int getInt(byte[] bArr, long j);

    /* access modifiers changed from: package-private */
    public abstract long getLong(long j);

    /* access modifiers changed from: package-private */
    public abstract long getLong(byte[] bArr, long j);

    /* access modifiers changed from: package-private */
    public abstract short getShort(long j);

    /* access modifiers changed from: package-private */
    public abstract short getShort(byte[] bArr, long j);

    /* access modifiers changed from: package-private */
    public abstract void putBoolean(long j, boolean z);

    /* access modifiers changed from: package-private */
    public abstract void putBoolean(byte[] bArr, long j, boolean z);

    /* access modifiers changed from: package-private */
    public abstract void putByte(long j, byte b);

    /* access modifiers changed from: package-private */
    public abstract void putByte(byte[] bArr, long j, byte b);

    /* access modifiers changed from: package-private */
    public abstract void putChar(long j, char c);

    /* access modifiers changed from: package-private */
    public abstract void putChar(byte[] bArr, long j, char c);

    /* access modifiers changed from: package-private */
    public abstract void putDouble(long j, double d);

    /* access modifiers changed from: package-private */
    public abstract void putDouble(byte[] bArr, long j, double d);

    /* access modifiers changed from: package-private */
    public abstract void putFloat(long j, float f);

    /* access modifiers changed from: package-private */
    public abstract void putFloat(byte[] bArr, long j, float f);

    /* access modifiers changed from: package-private */
    public abstract void putInt(long j, int i);

    /* access modifiers changed from: package-private */
    public abstract void putInt(byte[] bArr, long j, int i);

    /* access modifiers changed from: package-private */
    public abstract void putLong(long j, long j2);

    /* access modifiers changed from: package-private */
    public abstract void putLong(byte[] bArr, long j, long j2);

    /* access modifiers changed from: package-private */
    public abstract void putShort(long j, short s);

    /* access modifiers changed from: package-private */
    public abstract void putShort(byte[] bArr, long j, short s);

    Raw() {
    }

    static {
        if (UnsafeRaw.isAvailable()) {
            INSTANCE = new UnsafeRaw();
        } else {
            INSTANCE = null;
        }
    }

    static Raw getInstance() {
        return INSTANCE;
    }
}
