package org.opencv.xphoto;

public class SimpleWB extends WhiteBalancer {
    private static native void delete(long j);

    private static native float getInputMax_0(long j);

    private static native float getInputMin_0(long j);

    private static native float getOutputMax_0(long j);

    private static native float getOutputMin_0(long j);

    private static native float getP_0(long j);

    private static native void setInputMax_0(long j, float f);

    private static native void setInputMin_0(long j, float f);

    private static native void setOutputMax_0(long j, float f);

    private static native void setOutputMin_0(long j, float f);

    private static native void setP_0(long j, float f);

    protected SimpleWB(long addr) {
        super(addr);
    }

    public static SimpleWB __fromPtr__(long addr) {
        return new SimpleWB(addr);
    }

    public float getInputMax() {
        return getInputMax_0(this.nativeObj);
    }

    public float getInputMin() {
        return getInputMin_0(this.nativeObj);
    }

    public float getOutputMax() {
        return getOutputMax_0(this.nativeObj);
    }

    public float getOutputMin() {
        return getOutputMin_0(this.nativeObj);
    }

    public float getP() {
        return getP_0(this.nativeObj);
    }

    public void setInputMax(float val) {
        setInputMax_0(this.nativeObj, val);
    }

    public void setInputMin(float val) {
        setInputMin_0(this.nativeObj, val);
    }

    public void setOutputMax(float val) {
        setOutputMax_0(this.nativeObj, val);
    }

    public void setOutputMin(float val) {
        setOutputMin_0(this.nativeObj, val);
    }

    public void setP(float val) {
        setP_0(this.nativeObj, val);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
