package org.opencv.img_hash;

import org.opencv.core.Mat;

public class Img_hash {
    public static final int BLOCK_MEAN_HASH_MODE_0 = 0;
    public static final int BLOCK_MEAN_HASH_MODE_1 = 1;

    private static native void averageHash_0(long j, long j2);

    private static native void blockMeanHash_0(long j, long j2, int i);

    private static native void blockMeanHash_1(long j, long j2);

    private static native void colorMomentHash_0(long j, long j2);

    private static native void marrHildrethHash_0(long j, long j2, float f, float f2);

    private static native void marrHildrethHash_1(long j, long j2, float f);

    private static native void marrHildrethHash_2(long j, long j2);

    private static native void pHash_0(long j, long j2);

    private static native void radialVarianceHash_0(long j, long j2, double d, int i);

    private static native void radialVarianceHash_1(long j, long j2, double d);

    private static native void radialVarianceHash_2(long j, long j2);

    public static void averageHash(Mat inputArr, Mat outputArr) {
        averageHash_0(inputArr.nativeObj, outputArr.nativeObj);
    }

    public static void blockMeanHash(Mat inputArr, Mat outputArr, int mode) {
        blockMeanHash_0(inputArr.nativeObj, outputArr.nativeObj, mode);
    }

    public static void blockMeanHash(Mat inputArr, Mat outputArr) {
        blockMeanHash_1(inputArr.nativeObj, outputArr.nativeObj);
    }

    public static void colorMomentHash(Mat inputArr, Mat outputArr) {
        colorMomentHash_0(inputArr.nativeObj, outputArr.nativeObj);
    }

    public static void marrHildrethHash(Mat inputArr, Mat outputArr, float alpha, float scale) {
        marrHildrethHash_0(inputArr.nativeObj, outputArr.nativeObj, alpha, scale);
    }

    public static void marrHildrethHash(Mat inputArr, Mat outputArr, float alpha) {
        marrHildrethHash_1(inputArr.nativeObj, outputArr.nativeObj, alpha);
    }

    public static void marrHildrethHash(Mat inputArr, Mat outputArr) {
        marrHildrethHash_2(inputArr.nativeObj, outputArr.nativeObj);
    }

    public static void pHash(Mat inputArr, Mat outputArr) {
        pHash_0(inputArr.nativeObj, outputArr.nativeObj);
    }

    public static void radialVarianceHash(Mat inputArr, Mat outputArr, double sigma, int numOfAngleLine) {
        radialVarianceHash_0(inputArr.nativeObj, outputArr.nativeObj, sigma, numOfAngleLine);
    }

    public static void radialVarianceHash(Mat inputArr, Mat outputArr, double sigma) {
        radialVarianceHash_1(inputArr.nativeObj, outputArr.nativeObj, sigma);
    }

    public static void radialVarianceHash(Mat inputArr, Mat outputArr) {
        radialVarianceHash_2(inputArr.nativeObj, outputArr.nativeObj);
    }
}
