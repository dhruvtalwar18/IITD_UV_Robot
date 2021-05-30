package org.opencv.video;

import org.opencv.core.Mat;

public class VariationalRefinement extends DenseOpticalFlow {
    private static native void calcUV_0(long j, long j2, long j3, long j4, long j5);

    private static native long create_0();

    private static native void delete(long j);

    private static native float getAlpha_0(long j);

    private static native float getDelta_0(long j);

    private static native int getFixedPointIterations_0(long j);

    private static native float getGamma_0(long j);

    private static native float getOmega_0(long j);

    private static native int getSorIterations_0(long j);

    private static native void setAlpha_0(long j, float f);

    private static native void setDelta_0(long j, float f);

    private static native void setFixedPointIterations_0(long j, int i);

    private static native void setGamma_0(long j, float f);

    private static native void setOmega_0(long j, float f);

    private static native void setSorIterations_0(long j, int i);

    protected VariationalRefinement(long addr) {
        super(addr);
    }

    public static VariationalRefinement __fromPtr__(long addr) {
        return new VariationalRefinement(addr);
    }

    public static VariationalRefinement create() {
        return __fromPtr__(create_0());
    }

    public float getAlpha() {
        return getAlpha_0(this.nativeObj);
    }

    public float getDelta() {
        return getDelta_0(this.nativeObj);
    }

    public float getGamma() {
        return getGamma_0(this.nativeObj);
    }

    public float getOmega() {
        return getOmega_0(this.nativeObj);
    }

    public int getFixedPointIterations() {
        return getFixedPointIterations_0(this.nativeObj);
    }

    public int getSorIterations() {
        return getSorIterations_0(this.nativeObj);
    }

    public void calcUV(Mat I0, Mat I1, Mat flow_u, Mat flow_v) {
        calcUV_0(this.nativeObj, I0.nativeObj, I1.nativeObj, flow_u.nativeObj, flow_v.nativeObj);
    }

    public void setAlpha(float val) {
        setAlpha_0(this.nativeObj, val);
    }

    public void setDelta(float val) {
        setDelta_0(this.nativeObj, val);
    }

    public void setFixedPointIterations(int val) {
        setFixedPointIterations_0(this.nativeObj, val);
    }

    public void setGamma(float val) {
        setGamma_0(this.nativeObj, val);
    }

    public void setOmega(float val) {
        setOmega_0(this.nativeObj, val);
    }

    public void setSorIterations(int val) {
        setSorIterations_0(this.nativeObj, val);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
