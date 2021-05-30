package org.opencv.xfeatures2d;

import org.opencv.features2d.Feature2D;

public class StarDetector extends Feature2D {
    private static native long create_0(int i, int i2, int i3, int i4, int i5);

    private static native long create_1(int i, int i2, int i3, int i4);

    private static native long create_2(int i, int i2, int i3);

    private static native long create_3(int i, int i2);

    private static native long create_4(int i);

    private static native long create_5();

    private static native void delete(long j);

    protected StarDetector(long addr) {
        super(addr);
    }

    public static StarDetector __fromPtr__(long addr) {
        return new StarDetector(addr);
    }

    public static StarDetector create(int maxSize, int responseThreshold, int lineThresholdProjected, int lineThresholdBinarized, int suppressNonmaxSize) {
        return __fromPtr__(create_0(maxSize, responseThreshold, lineThresholdProjected, lineThresholdBinarized, suppressNonmaxSize));
    }

    public static StarDetector create(int maxSize, int responseThreshold, int lineThresholdProjected, int lineThresholdBinarized) {
        return __fromPtr__(create_1(maxSize, responseThreshold, lineThresholdProjected, lineThresholdBinarized));
    }

    public static StarDetector create(int maxSize, int responseThreshold, int lineThresholdProjected) {
        return __fromPtr__(create_2(maxSize, responseThreshold, lineThresholdProjected));
    }

    public static StarDetector create(int maxSize, int responseThreshold) {
        return __fromPtr__(create_3(maxSize, responseThreshold));
    }

    public static StarDetector create(int maxSize) {
        return __fromPtr__(create_4(maxSize));
    }

    public static StarDetector create() {
        return __fromPtr__(create_5());
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
