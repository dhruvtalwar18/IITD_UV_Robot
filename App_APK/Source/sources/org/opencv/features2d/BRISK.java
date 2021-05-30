package org.opencv.features2d;

import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;

public class BRISK extends Feature2D {
    private static native long create_0(int i, int i2, long j, long j2, float f, float f2, long j3);

    private static native long create_1(int i, int i2, long j, long j2, float f, float f2);

    private static native long create_10(long j, long j2, float f);

    private static native long create_11(long j, long j2);

    private static native long create_2(int i, int i2, long j, long j2, float f);

    private static native long create_3(int i, int i2, long j, long j2);

    private static native long create_4(int i, int i2, float f);

    private static native long create_5(int i, int i2);

    private static native long create_6(int i);

    private static native long create_7();

    private static native long create_8(long j, long j2, float f, float f2, long j3);

    private static native long create_9(long j, long j2, float f, float f2);

    private static native void delete(long j);

    private static native String getDefaultName_0(long j);

    private static native int getOctaves_0(long j);

    private static native int getThreshold_0(long j);

    private static native void setOctaves_0(long j, int i);

    private static native void setThreshold_0(long j, int i);

    protected BRISK(long addr) {
        super(addr);
    }

    public static BRISK __fromPtr__(long addr) {
        return new BRISK(addr);
    }

    public static BRISK create(int thresh, int octaves, MatOfFloat radiusList, MatOfInt numberList, float dMax, float dMin, MatOfInt indexChange) {
        return __fromPtr__(create_0(thresh, octaves, radiusList.nativeObj, numberList.nativeObj, dMax, dMin, indexChange.nativeObj));
    }

    public static BRISK create(int thresh, int octaves, MatOfFloat radiusList, MatOfInt numberList, float dMax, float dMin) {
        return __fromPtr__(create_1(thresh, octaves, radiusList.nativeObj, numberList.nativeObj, dMax, dMin));
    }

    public static BRISK create(int thresh, int octaves, MatOfFloat radiusList, MatOfInt numberList, float dMax) {
        return __fromPtr__(create_2(thresh, octaves, radiusList.nativeObj, numberList.nativeObj, dMax));
    }

    public static BRISK create(int thresh, int octaves, MatOfFloat radiusList, MatOfInt numberList) {
        return __fromPtr__(create_3(thresh, octaves, radiusList.nativeObj, numberList.nativeObj));
    }

    public static BRISK create(int thresh, int octaves, float patternScale) {
        return __fromPtr__(create_4(thresh, octaves, patternScale));
    }

    public static BRISK create(int thresh, int octaves) {
        return __fromPtr__(create_5(thresh, octaves));
    }

    public static BRISK create(int thresh) {
        return __fromPtr__(create_6(thresh));
    }

    public static BRISK create() {
        return __fromPtr__(create_7());
    }

    public static BRISK create(MatOfFloat radiusList, MatOfInt numberList, float dMax, float dMin, MatOfInt indexChange) {
        return __fromPtr__(create_8(radiusList.nativeObj, numberList.nativeObj, dMax, dMin, indexChange.nativeObj));
    }

    public static BRISK create(MatOfFloat radiusList, MatOfInt numberList, float dMax, float dMin) {
        return __fromPtr__(create_9(radiusList.nativeObj, numberList.nativeObj, dMax, dMin));
    }

    public static BRISK create(MatOfFloat radiusList, MatOfInt numberList, float dMax) {
        return __fromPtr__(create_10(radiusList.nativeObj, numberList.nativeObj, dMax));
    }

    public static BRISK create(MatOfFloat radiusList, MatOfInt numberList) {
        return __fromPtr__(create_11(radiusList.nativeObj, numberList.nativeObj));
    }

    public String getDefaultName() {
        return getDefaultName_0(this.nativeObj);
    }

    public int getOctaves() {
        return getOctaves_0(this.nativeObj);
    }

    public int getThreshold() {
        return getThreshold_0(this.nativeObj);
    }

    public void setOctaves(int octaves) {
        setOctaves_0(this.nativeObj, octaves);
    }

    public void setThreshold(int threshold) {
        setThreshold_0(this.nativeObj, threshold);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
