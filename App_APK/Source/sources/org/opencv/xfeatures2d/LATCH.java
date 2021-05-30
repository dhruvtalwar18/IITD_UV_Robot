package org.opencv.xfeatures2d;

import org.opencv.features2d.Feature2D;

public class LATCH extends Feature2D {
    private static native long create_0(int i, boolean z, int i2, double d);

    private static native long create_1(int i, boolean z, int i2);

    private static native long create_2(int i, boolean z);

    private static native long create_3(int i);

    private static native long create_4();

    private static native void delete(long j);

    protected LATCH(long addr) {
        super(addr);
    }

    public static LATCH __fromPtr__(long addr) {
        return new LATCH(addr);
    }

    public static LATCH create(int bytes, boolean rotationInvariance, int half_ssd_size, double sigma) {
        return __fromPtr__(create_0(bytes, rotationInvariance, half_ssd_size, sigma));
    }

    public static LATCH create(int bytes, boolean rotationInvariance, int half_ssd_size) {
        return __fromPtr__(create_1(bytes, rotationInvariance, half_ssd_size));
    }

    public static LATCH create(int bytes, boolean rotationInvariance) {
        return __fromPtr__(create_2(bytes, rotationInvariance));
    }

    public static LATCH create(int bytes) {
        return __fromPtr__(create_3(bytes));
    }

    public static LATCH create() {
        return __fromPtr__(create_4());
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
