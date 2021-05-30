package org.opencv.xfeatures2d;

import org.opencv.features2d.Feature2D;

public class HarrisLaplaceFeatureDetector extends Feature2D {
    private static native long create_0(int i, float f, float f2, int i2, int i3);

    private static native long create_1(int i, float f, float f2, int i2);

    private static native long create_2(int i, float f, float f2);

    private static native long create_3(int i, float f);

    private static native long create_4(int i);

    private static native long create_5();

    private static native void delete(long j);

    protected HarrisLaplaceFeatureDetector(long addr) {
        super(addr);
    }

    public static HarrisLaplaceFeatureDetector __fromPtr__(long addr) {
        return new HarrisLaplaceFeatureDetector(addr);
    }

    public static HarrisLaplaceFeatureDetector create(int numOctaves, float corn_thresh, float DOG_thresh, int maxCorners, int num_layers) {
        return __fromPtr__(create_0(numOctaves, corn_thresh, DOG_thresh, maxCorners, num_layers));
    }

    public static HarrisLaplaceFeatureDetector create(int numOctaves, float corn_thresh, float DOG_thresh, int maxCorners) {
        return __fromPtr__(create_1(numOctaves, corn_thresh, DOG_thresh, maxCorners));
    }

    public static HarrisLaplaceFeatureDetector create(int numOctaves, float corn_thresh, float DOG_thresh) {
        return __fromPtr__(create_2(numOctaves, corn_thresh, DOG_thresh));
    }

    public static HarrisLaplaceFeatureDetector create(int numOctaves, float corn_thresh) {
        return __fromPtr__(create_3(numOctaves, corn_thresh));
    }

    public static HarrisLaplaceFeatureDetector create(int numOctaves) {
        return __fromPtr__(create_4(numOctaves));
    }

    public static HarrisLaplaceFeatureDetector create() {
        return __fromPtr__(create_5());
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
