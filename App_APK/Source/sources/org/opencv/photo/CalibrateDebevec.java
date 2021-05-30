package org.opencv.photo;

public class CalibrateDebevec extends CalibrateCRF {
    private static native void delete(long j);

    private static native float getLambda_0(long j);

    private static native boolean getRandom_0(long j);

    private static native int getSamples_0(long j);

    private static native void setLambda_0(long j, float f);

    private static native void setRandom_0(long j, boolean z);

    private static native void setSamples_0(long j, int i);

    protected CalibrateDebevec(long addr) {
        super(addr);
    }

    public static CalibrateDebevec __fromPtr__(long addr) {
        return new CalibrateDebevec(addr);
    }

    public boolean getRandom() {
        return getRandom_0(this.nativeObj);
    }

    public float getLambda() {
        return getLambda_0(this.nativeObj);
    }

    public int getSamples() {
        return getSamples_0(this.nativeObj);
    }

    public void setLambda(float lambda) {
        setLambda_0(this.nativeObj, lambda);
    }

    public void setRandom(boolean random) {
        setRandom_0(this.nativeObj, random);
    }

    public void setSamples(int samples) {
        setSamples_0(this.nativeObj, samples);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
