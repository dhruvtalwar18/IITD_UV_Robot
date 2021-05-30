package org.opencv.ml;

public class ParamGrid {
    protected final long nativeObj;

    private static native long create_0(double d, double d2, double d3);

    private static native long create_1(double d, double d2);

    private static native long create_2(double d);

    private static native long create_3();

    private static native void delete(long j);

    private static native double get_logStep_0(long j);

    private static native double get_maxVal_0(long j);

    private static native double get_minVal_0(long j);

    private static native void set_logStep_0(long j, double d);

    private static native void set_maxVal_0(long j, double d);

    private static native void set_minVal_0(long j, double d);

    protected ParamGrid(long addr) {
        this.nativeObj = addr;
    }

    public long getNativeObjAddr() {
        return this.nativeObj;
    }

    public static ParamGrid __fromPtr__(long addr) {
        return new ParamGrid(addr);
    }

    public static ParamGrid create(double minVal, double maxVal, double logstep) {
        return __fromPtr__(create_0(minVal, maxVal, logstep));
    }

    public static ParamGrid create(double minVal, double maxVal) {
        return __fromPtr__(create_1(minVal, maxVal));
    }

    public static ParamGrid create(double minVal) {
        return __fromPtr__(create_2(minVal));
    }

    public static ParamGrid create() {
        return __fromPtr__(create_3());
    }

    public double get_minVal() {
        return get_minVal_0(this.nativeObj);
    }

    public void set_minVal(double minVal) {
        set_minVal_0(this.nativeObj, minVal);
    }

    public double get_maxVal() {
        return get_maxVal_0(this.nativeObj);
    }

    public void set_maxVal(double maxVal) {
        set_maxVal_0(this.nativeObj, maxVal);
    }

    public double get_logStep() {
        return get_logStep_0(this.nativeObj);
    }

    public void set_logStep(double logStep) {
        set_logStep_0(this.nativeObj, logStep);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
