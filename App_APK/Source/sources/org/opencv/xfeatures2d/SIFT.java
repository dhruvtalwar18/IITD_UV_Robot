package org.opencv.xfeatures2d;

import org.opencv.features2d.Feature2D;

public class SIFT extends Feature2D {
    private static native long create_0(int i, int i2, double d, double d2, double d3);

    private static native long create_1(int i, int i2, double d, double d2);

    private static native long create_2(int i, int i2, double d);

    private static native long create_3(int i, int i2);

    private static native long create_4(int i);

    private static native long create_5();

    private static native void delete(long j);

    protected SIFT(long addr) {
        super(addr);
    }

    public static SIFT __fromPtr__(long addr) {
        return new SIFT(addr);
    }

    public static SIFT create(int nfeatures, int nOctaveLayers, double contrastThreshold, double edgeThreshold, double sigma) {
        return __fromPtr__(create_0(nfeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma));
    }

    public static SIFT create(int nfeatures, int nOctaveLayers, double contrastThreshold, double edgeThreshold) {
        return __fromPtr__(create_1(nfeatures, nOctaveLayers, contrastThreshold, edgeThreshold));
    }

    public static SIFT create(int nfeatures, int nOctaveLayers, double contrastThreshold) {
        return __fromPtr__(create_2(nfeatures, nOctaveLayers, contrastThreshold));
    }

    public static SIFT create(int nfeatures, int nOctaveLayers) {
        return __fromPtr__(create_3(nfeatures, nOctaveLayers));
    }

    public static SIFT create(int nfeatures) {
        return __fromPtr__(create_4(nfeatures));
    }

    public static SIFT create() {
        return __fromPtr__(create_5());
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
