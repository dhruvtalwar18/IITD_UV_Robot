package org.opencv.xfeatures2d;

import org.opencv.core.MatOfInt;
import org.opencv.features2d.Feature2D;

public class FREAK extends Feature2D {
    private static native long create_0(boolean z, boolean z2, float f, int i, long j);

    private static native long create_1(boolean z, boolean z2, float f, int i);

    private static native long create_2(boolean z, boolean z2, float f);

    private static native long create_3(boolean z, boolean z2);

    private static native long create_4(boolean z);

    private static native long create_5();

    private static native void delete(long j);

    protected FREAK(long addr) {
        super(addr);
    }

    public static FREAK __fromPtr__(long addr) {
        return new FREAK(addr);
    }

    public static FREAK create(boolean orientationNormalized, boolean scaleNormalized, float patternScale, int nOctaves, MatOfInt selectedPairs) {
        return __fromPtr__(create_0(orientationNormalized, scaleNormalized, patternScale, nOctaves, selectedPairs.nativeObj));
    }

    public static FREAK create(boolean orientationNormalized, boolean scaleNormalized, float patternScale, int nOctaves) {
        return __fromPtr__(create_1(orientationNormalized, scaleNormalized, patternScale, nOctaves));
    }

    public static FREAK create(boolean orientationNormalized, boolean scaleNormalized, float patternScale) {
        return __fromPtr__(create_2(orientationNormalized, scaleNormalized, patternScale));
    }

    public static FREAK create(boolean orientationNormalized, boolean scaleNormalized) {
        return __fromPtr__(create_3(orientationNormalized, scaleNormalized));
    }

    public static FREAK create(boolean orientationNormalized) {
        return __fromPtr__(create_4(orientationNormalized));
    }

    public static FREAK create() {
        return __fromPtr__(create_5());
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
