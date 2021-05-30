package org.opencv.xfeatures2d;

import org.opencv.features2d.Feature2D;

public class LUCID extends Feature2D {
    private static native long create_0(int i, int i2);

    private static native long create_1(int i);

    private static native long create_2();

    private static native void delete(long j);

    protected LUCID(long addr) {
        super(addr);
    }

    public static LUCID __fromPtr__(long addr) {
        return new LUCID(addr);
    }

    public static LUCID create(int lucid_kernel, int blur_kernel) {
        return __fromPtr__(create_0(lucid_kernel, blur_kernel));
    }

    public static LUCID create(int lucid_kernel) {
        return __fromPtr__(create_1(lucid_kernel));
    }

    public static LUCID create() {
        return __fromPtr__(create_2());
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
