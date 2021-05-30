package org.opencv.xfeatures2d;

import org.opencv.features2d.Feature2D;

public class BoostDesc extends Feature2D {
    private static native long create_0(int i, boolean z, float f);

    private static native long create_1(int i, boolean z);

    private static native long create_2(int i);

    private static native long create_3();

    private static native void delete(long j);

    private static native float getScaleFactor_0(long j);

    private static native boolean getUseScaleOrientation_0(long j);

    private static native void setScaleFactor_0(long j, float f);

    private static native void setUseScaleOrientation_0(long j, boolean z);

    protected BoostDesc(long addr) {
        super(addr);
    }

    public static BoostDesc __fromPtr__(long addr) {
        return new BoostDesc(addr);
    }

    public static BoostDesc create(int desc, boolean use_scale_orientation, float scale_factor) {
        return __fromPtr__(create_0(desc, use_scale_orientation, scale_factor));
    }

    public static BoostDesc create(int desc, boolean use_scale_orientation) {
        return __fromPtr__(create_1(desc, use_scale_orientation));
    }

    public static BoostDesc create(int desc) {
        return __fromPtr__(create_2(desc));
    }

    public static BoostDesc create() {
        return __fromPtr__(create_3());
    }

    public boolean getUseScaleOrientation() {
        return getUseScaleOrientation_0(this.nativeObj);
    }

    public float getScaleFactor() {
        return getScaleFactor_0(this.nativeObj);
    }

    public void setScaleFactor(float scale_factor) {
        setScaleFactor_0(this.nativeObj, scale_factor);
    }

    public void setUseScaleOrientation(boolean use_scale_orientation) {
        setUseScaleOrientation_0(this.nativeObj, use_scale_orientation);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
