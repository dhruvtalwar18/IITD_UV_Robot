package org.opencv.xfeatures2d;

import org.opencv.features2d.Feature2D;

public class SURF extends Feature2D {
    private static native long create_0(double d, int i, int i2, boolean z, boolean z2);

    private static native long create_1(double d, int i, int i2, boolean z);

    private static native long create_2(double d, int i, int i2);

    private static native long create_3(double d, int i);

    private static native long create_4(double d);

    private static native long create_5();

    private static native void delete(long j);

    private static native boolean getExtended_0(long j);

    private static native double getHessianThreshold_0(long j);

    private static native int getNOctaveLayers_0(long j);

    private static native int getNOctaves_0(long j);

    private static native boolean getUpright_0(long j);

    private static native void setExtended_0(long j, boolean z);

    private static native void setHessianThreshold_0(long j, double d);

    private static native void setNOctaveLayers_0(long j, int i);

    private static native void setNOctaves_0(long j, int i);

    private static native void setUpright_0(long j, boolean z);

    protected SURF(long addr) {
        super(addr);
    }

    public static SURF __fromPtr__(long addr) {
        return new SURF(addr);
    }

    public static SURF create(double hessianThreshold, int nOctaves, int nOctaveLayers, boolean extended, boolean upright) {
        return __fromPtr__(create_0(hessianThreshold, nOctaves, nOctaveLayers, extended, upright));
    }

    public static SURF create(double hessianThreshold, int nOctaves, int nOctaveLayers, boolean extended) {
        return __fromPtr__(create_1(hessianThreshold, nOctaves, nOctaveLayers, extended));
    }

    public static SURF create(double hessianThreshold, int nOctaves, int nOctaveLayers) {
        return __fromPtr__(create_2(hessianThreshold, nOctaves, nOctaveLayers));
    }

    public static SURF create(double hessianThreshold, int nOctaves) {
        return __fromPtr__(create_3(hessianThreshold, nOctaves));
    }

    public static SURF create(double hessianThreshold) {
        return __fromPtr__(create_4(hessianThreshold));
    }

    public static SURF create() {
        return __fromPtr__(create_5());
    }

    public boolean getExtended() {
        return getExtended_0(this.nativeObj);
    }

    public boolean getUpright() {
        return getUpright_0(this.nativeObj);
    }

    public double getHessianThreshold() {
        return getHessianThreshold_0(this.nativeObj);
    }

    public int getNOctaveLayers() {
        return getNOctaveLayers_0(this.nativeObj);
    }

    public int getNOctaves() {
        return getNOctaves_0(this.nativeObj);
    }

    public void setExtended(boolean extended) {
        setExtended_0(this.nativeObj, extended);
    }

    public void setHessianThreshold(double hessianThreshold) {
        setHessianThreshold_0(this.nativeObj, hessianThreshold);
    }

    public void setNOctaveLayers(int nOctaveLayers) {
        setNOctaveLayers_0(this.nativeObj, nOctaveLayers);
    }

    public void setNOctaves(int nOctaves) {
        setNOctaves_0(this.nativeObj, nOctaves);
    }

    public void setUpright(boolean upright) {
        setUpright_0(this.nativeObj, upright);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
