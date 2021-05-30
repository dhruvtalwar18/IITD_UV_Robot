package org.opencv.features2d;

public class KAZE extends Feature2D {
    public static final int DIFF_CHARBONNIER = 3;
    public static final int DIFF_PM_G1 = 0;
    public static final int DIFF_PM_G2 = 1;
    public static final int DIFF_WEICKERT = 2;

    private static native long create_0(boolean z, boolean z2, float f, int i, int i2, int i3);

    private static native long create_1(boolean z, boolean z2, float f, int i, int i2);

    private static native long create_2(boolean z, boolean z2, float f, int i);

    private static native long create_3(boolean z, boolean z2, float f);

    private static native long create_4(boolean z, boolean z2);

    private static native long create_5(boolean z);

    private static native long create_6();

    private static native void delete(long j);

    private static native String getDefaultName_0(long j);

    private static native int getDiffusivity_0(long j);

    private static native boolean getExtended_0(long j);

    private static native int getNOctaveLayers_0(long j);

    private static native int getNOctaves_0(long j);

    private static native double getThreshold_0(long j);

    private static native boolean getUpright_0(long j);

    private static native void setDiffusivity_0(long j, int i);

    private static native void setExtended_0(long j, boolean z);

    private static native void setNOctaveLayers_0(long j, int i);

    private static native void setNOctaves_0(long j, int i);

    private static native void setThreshold_0(long j, double d);

    private static native void setUpright_0(long j, boolean z);

    protected KAZE(long addr) {
        super(addr);
    }

    public static KAZE __fromPtr__(long addr) {
        return new KAZE(addr);
    }

    public int getDiffusivity() {
        return getDiffusivity_0(this.nativeObj);
    }

    public static KAZE create(boolean extended, boolean upright, float threshold, int nOctaves, int nOctaveLayers, int diffusivity) {
        return __fromPtr__(create_0(extended, upright, threshold, nOctaves, nOctaveLayers, diffusivity));
    }

    public static KAZE create(boolean extended, boolean upright, float threshold, int nOctaves, int nOctaveLayers) {
        return __fromPtr__(create_1(extended, upright, threshold, nOctaves, nOctaveLayers));
    }

    public static KAZE create(boolean extended, boolean upright, float threshold, int nOctaves) {
        return __fromPtr__(create_2(extended, upright, threshold, nOctaves));
    }

    public static KAZE create(boolean extended, boolean upright, float threshold) {
        return __fromPtr__(create_3(extended, upright, threshold));
    }

    public static KAZE create(boolean extended, boolean upright) {
        return __fromPtr__(create_4(extended, upright));
    }

    public static KAZE create(boolean extended) {
        return __fromPtr__(create_5(extended));
    }

    public static KAZE create() {
        return __fromPtr__(create_6());
    }

    public String getDefaultName() {
        return getDefaultName_0(this.nativeObj);
    }

    public boolean getExtended() {
        return getExtended_0(this.nativeObj);
    }

    public boolean getUpright() {
        return getUpright_0(this.nativeObj);
    }

    public double getThreshold() {
        return getThreshold_0(this.nativeObj);
    }

    public int getNOctaveLayers() {
        return getNOctaveLayers_0(this.nativeObj);
    }

    public int getNOctaves() {
        return getNOctaves_0(this.nativeObj);
    }

    public void setDiffusivity(int diff) {
        setDiffusivity_0(this.nativeObj, diff);
    }

    public void setExtended(boolean extended) {
        setExtended_0(this.nativeObj, extended);
    }

    public void setNOctaveLayers(int octaveLayers) {
        setNOctaveLayers_0(this.nativeObj, octaveLayers);
    }

    public void setNOctaves(int octaves) {
        setNOctaves_0(this.nativeObj, octaves);
    }

    public void setThreshold(double threshold) {
        setThreshold_0(this.nativeObj, threshold);
    }

    public void setUpright(boolean upright) {
        setUpright_0(this.nativeObj, upright);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
