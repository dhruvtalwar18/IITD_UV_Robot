package org.opencv.xfeatures2d;

import org.opencv.core.Mat;
import org.opencv.features2d.Feature2D;

public class DAISY extends Feature2D {
    public static final int NRM_FULL = 102;
    public static final int NRM_NONE = 100;
    public static final int NRM_PARTIAL = 101;
    public static final int NRM_SIFT = 103;

    private static native long create_0(float f, int i, int i2, int i3, long j, boolean z, boolean z2);

    private static native long create_1(float f, int i, int i2, int i3, long j, boolean z);

    private static native long create_2(float f, int i, int i2, int i3, long j);

    private static native long create_3(float f, int i, int i2, int i3);

    private static native long create_5(float f, int i, int i2);

    private static native long create_6(float f, int i);

    private static native long create_7(float f);

    private static native long create_8();

    private static native void delete(long j);

    protected DAISY(long addr) {
        super(addr);
    }

    public static DAISY __fromPtr__(long addr) {
        return new DAISY(addr);
    }

    public static DAISY create(float radius, int q_radius, int q_theta, int q_hist, Mat H, boolean interpolation, boolean use_orientation) {
        return __fromPtr__(create_0(radius, q_radius, q_theta, q_hist, H.nativeObj, interpolation, use_orientation));
    }

    public static DAISY create(float radius, int q_radius, int q_theta, int q_hist, Mat H, boolean interpolation) {
        return __fromPtr__(create_1(radius, q_radius, q_theta, q_hist, H.nativeObj, interpolation));
    }

    public static DAISY create(float radius, int q_radius, int q_theta, int q_hist, Mat H) {
        return __fromPtr__(create_2(radius, q_radius, q_theta, q_hist, H.nativeObj));
    }

    public static DAISY create(float radius, int q_radius, int q_theta, int q_hist) {
        return __fromPtr__(create_3(radius, q_radius, q_theta, q_hist));
    }

    public static DAISY create(float radius, int q_radius, int q_theta) {
        return __fromPtr__(create_5(radius, q_radius, q_theta));
    }

    public static DAISY create(float radius, int q_radius) {
        return __fromPtr__(create_6(radius, q_radius));
    }

    public static DAISY create(float radius) {
        return __fromPtr__(create_7(radius));
    }

    public static DAISY create() {
        return __fromPtr__(create_8());
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
