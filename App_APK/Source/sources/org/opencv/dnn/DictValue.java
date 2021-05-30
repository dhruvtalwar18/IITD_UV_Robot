package org.opencv.dnn;

public class DictValue {
    protected final long nativeObj;

    private static native long DictValue_0(String str);

    private static native long DictValue_1(double d);

    private static native long DictValue_2(int i);

    private static native void delete(long j);

    private static native int getIntValue_0(long j, int i);

    private static native int getIntValue_1(long j);

    private static native double getRealValue_0(long j, int i);

    private static native double getRealValue_1(long j);

    private static native String getStringValue_0(long j, int i);

    private static native String getStringValue_1(long j);

    private static native boolean isInt_0(long j);

    private static native boolean isReal_0(long j);

    private static native boolean isString_0(long j);

    protected DictValue(long addr) {
        this.nativeObj = addr;
    }

    public long getNativeObjAddr() {
        return this.nativeObj;
    }

    public static DictValue __fromPtr__(long addr) {
        return new DictValue(addr);
    }

    public DictValue(String s) {
        this.nativeObj = DictValue_0(s);
    }

    public DictValue(double p) {
        this.nativeObj = DictValue_1(p);
    }

    public DictValue(int i) {
        this.nativeObj = DictValue_2(i);
    }

    public String getStringValue(int idx) {
        return getStringValue_0(this.nativeObj, idx);
    }

    public String getStringValue() {
        return getStringValue_1(this.nativeObj);
    }

    public boolean isInt() {
        return isInt_0(this.nativeObj);
    }

    public boolean isReal() {
        return isReal_0(this.nativeObj);
    }

    public boolean isString() {
        return isString_0(this.nativeObj);
    }

    public double getRealValue(int idx) {
        return getRealValue_0(this.nativeObj, idx);
    }

    public double getRealValue() {
        return getRealValue_1(this.nativeObj);
    }

    public int getIntValue(int idx) {
        return getIntValue_0(this.nativeObj, idx);
    }

    public int getIntValue() {
        return getIntValue_1(this.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
