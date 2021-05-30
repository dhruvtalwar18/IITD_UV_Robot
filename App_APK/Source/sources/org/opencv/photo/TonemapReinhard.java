package org.opencv.photo;

public class TonemapReinhard extends Tonemap {
    private static native void delete(long j);

    private static native float getColorAdaptation_0(long j);

    private static native float getIntensity_0(long j);

    private static native float getLightAdaptation_0(long j);

    private static native void setColorAdaptation_0(long j, float f);

    private static native void setIntensity_0(long j, float f);

    private static native void setLightAdaptation_0(long j, float f);

    protected TonemapReinhard(long addr) {
        super(addr);
    }

    public static TonemapReinhard __fromPtr__(long addr) {
        return new TonemapReinhard(addr);
    }

    public float getColorAdaptation() {
        return getColorAdaptation_0(this.nativeObj);
    }

    public float getIntensity() {
        return getIntensity_0(this.nativeObj);
    }

    public float getLightAdaptation() {
        return getLightAdaptation_0(this.nativeObj);
    }

    public void setColorAdaptation(float color_adapt) {
        setColorAdaptation_0(this.nativeObj, color_adapt);
    }

    public void setIntensity(float intensity) {
        setIntensity_0(this.nativeObj, intensity);
    }

    public void setLightAdaptation(float light_adapt) {
        setLightAdaptation_0(this.nativeObj, light_adapt);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
