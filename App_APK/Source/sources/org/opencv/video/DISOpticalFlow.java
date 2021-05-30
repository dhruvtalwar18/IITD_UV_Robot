package org.opencv.video;

public class DISOpticalFlow extends DenseOpticalFlow {
    public static final int PRESET_FAST = 1;
    public static final int PRESET_MEDIUM = 2;
    public static final int PRESET_ULTRAFAST = 0;

    private static native long create_0(int i);

    private static native long create_1();

    private static native void delete(long j);

    private static native int getFinestScale_0(long j);

    private static native int getGradientDescentIterations_0(long j);

    private static native int getPatchSize_0(long j);

    private static native int getPatchStride_0(long j);

    private static native boolean getUseMeanNormalization_0(long j);

    private static native boolean getUseSpatialPropagation_0(long j);

    private static native float getVariationalRefinementAlpha_0(long j);

    private static native float getVariationalRefinementDelta_0(long j);

    private static native float getVariationalRefinementGamma_0(long j);

    private static native int getVariationalRefinementIterations_0(long j);

    private static native void setFinestScale_0(long j, int i);

    private static native void setGradientDescentIterations_0(long j, int i);

    private static native void setPatchSize_0(long j, int i);

    private static native void setPatchStride_0(long j, int i);

    private static native void setUseMeanNormalization_0(long j, boolean z);

    private static native void setUseSpatialPropagation_0(long j, boolean z);

    private static native void setVariationalRefinementAlpha_0(long j, float f);

    private static native void setVariationalRefinementDelta_0(long j, float f);

    private static native void setVariationalRefinementGamma_0(long j, float f);

    private static native void setVariationalRefinementIterations_0(long j, int i);

    protected DISOpticalFlow(long addr) {
        super(addr);
    }

    public static DISOpticalFlow __fromPtr__(long addr) {
        return new DISOpticalFlow(addr);
    }

    public static DISOpticalFlow create(int preset) {
        return __fromPtr__(create_0(preset));
    }

    public static DISOpticalFlow create() {
        return __fromPtr__(create_1());
    }

    public boolean getUseMeanNormalization() {
        return getUseMeanNormalization_0(this.nativeObj);
    }

    public boolean getUseSpatialPropagation() {
        return getUseSpatialPropagation_0(this.nativeObj);
    }

    public float getVariationalRefinementAlpha() {
        return getVariationalRefinementAlpha_0(this.nativeObj);
    }

    public float getVariationalRefinementDelta() {
        return getVariationalRefinementDelta_0(this.nativeObj);
    }

    public float getVariationalRefinementGamma() {
        return getVariationalRefinementGamma_0(this.nativeObj);
    }

    public int getFinestScale() {
        return getFinestScale_0(this.nativeObj);
    }

    public int getGradientDescentIterations() {
        return getGradientDescentIterations_0(this.nativeObj);
    }

    public int getPatchSize() {
        return getPatchSize_0(this.nativeObj);
    }

    public int getPatchStride() {
        return getPatchStride_0(this.nativeObj);
    }

    public int getVariationalRefinementIterations() {
        return getVariationalRefinementIterations_0(this.nativeObj);
    }

    public void setFinestScale(int val) {
        setFinestScale_0(this.nativeObj, val);
    }

    public void setGradientDescentIterations(int val) {
        setGradientDescentIterations_0(this.nativeObj, val);
    }

    public void setPatchSize(int val) {
        setPatchSize_0(this.nativeObj, val);
    }

    public void setPatchStride(int val) {
        setPatchStride_0(this.nativeObj, val);
    }

    public void setUseMeanNormalization(boolean val) {
        setUseMeanNormalization_0(this.nativeObj, val);
    }

    public void setUseSpatialPropagation(boolean val) {
        setUseSpatialPropagation_0(this.nativeObj, val);
    }

    public void setVariationalRefinementAlpha(float val) {
        setVariationalRefinementAlpha_0(this.nativeObj, val);
    }

    public void setVariationalRefinementDelta(float val) {
        setVariationalRefinementDelta_0(this.nativeObj, val);
    }

    public void setVariationalRefinementGamma(float val) {
        setVariationalRefinementGamma_0(this.nativeObj, val);
    }

    public void setVariationalRefinementIterations(int val) {
        setVariationalRefinementIterations_0(this.nativeObj, val);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
