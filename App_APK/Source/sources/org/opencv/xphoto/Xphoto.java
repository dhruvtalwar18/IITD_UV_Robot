package org.opencv.xphoto;

import org.opencv.core.Mat;

public class Xphoto {
    public static final int BM3D_STEP1 = 1;
    public static final int BM3D_STEP2 = 2;
    public static final int BM3D_STEPALL = 0;
    public static final int HAAR = 0;
    public static final int INPAINT_SHIFTMAP = 0;

    private static native void applyChannelGains_0(long j, long j2, float f, float f2, float f3);

    private static native void bm3dDenoising_0(long j, long j2, float f, int i, int i2, int i3, int i4, int i5, int i6, float f2, int i7, int i8, int i9);

    private static native void bm3dDenoising_1(long j, long j2, float f, int i, int i2, int i3, int i4, int i5, int i6, float f2, int i7, int i8);

    private static native void bm3dDenoising_10(long j, long j2, float f);

    private static native void bm3dDenoising_11(long j, long j2);

    private static native void bm3dDenoising_12(long j, long j2, long j3, float f, int i, int i2, int i3, int i4, int i5, int i6, float f2, int i7, int i8, int i9);

    private static native void bm3dDenoising_13(long j, long j2, long j3, float f, int i, int i2, int i3, int i4, int i5, int i6, float f2, int i7, int i8);

    private static native void bm3dDenoising_14(long j, long j2, long j3, float f, int i, int i2, int i3, int i4, int i5, int i6, float f2, int i7);

    private static native void bm3dDenoising_15(long j, long j2, long j3, float f, int i, int i2, int i3, int i4, int i5, int i6, float f2);

    private static native void bm3dDenoising_16(long j, long j2, long j3, float f, int i, int i2, int i3, int i4, int i5, int i6);

    private static native void bm3dDenoising_17(long j, long j2, long j3, float f, int i, int i2, int i3, int i4, int i5);

    private static native void bm3dDenoising_18(long j, long j2, long j3, float f, int i, int i2, int i3, int i4);

    private static native void bm3dDenoising_19(long j, long j2, long j3, float f, int i, int i2, int i3);

    private static native void bm3dDenoising_2(long j, long j2, float f, int i, int i2, int i3, int i4, int i5, int i6, float f2, int i7);

    private static native void bm3dDenoising_20(long j, long j2, long j3, float f, int i, int i2);

    private static native void bm3dDenoising_21(long j, long j2, long j3, float f, int i);

    private static native void bm3dDenoising_22(long j, long j2, long j3, float f);

    private static native void bm3dDenoising_23(long j, long j2, long j3);

    private static native void bm3dDenoising_3(long j, long j2, float f, int i, int i2, int i3, int i4, int i5, int i6, float f2);

    private static native void bm3dDenoising_4(long j, long j2, float f, int i, int i2, int i3, int i4, int i5, int i6);

    private static native void bm3dDenoising_5(long j, long j2, float f, int i, int i2, int i3, int i4, int i5);

    private static native void bm3dDenoising_6(long j, long j2, float f, int i, int i2, int i3, int i4);

    private static native void bm3dDenoising_7(long j, long j2, float f, int i, int i2, int i3);

    private static native void bm3dDenoising_8(long j, long j2, float f, int i, int i2);

    private static native void bm3dDenoising_9(long j, long j2, float f, int i);

    private static native long createGrayworldWB_0();

    private static native long createLearningBasedWB_0(String str);

    private static native long createLearningBasedWB_1();

    private static native long createSimpleWB_0();

    private static native long createTonemapDurand_0(float f, float f2, float f3, float f4, float f5);

    private static native long createTonemapDurand_1(float f, float f2, float f3, float f4);

    private static native long createTonemapDurand_2(float f, float f2, float f3);

    private static native long createTonemapDurand_3(float f, float f2);

    private static native long createTonemapDurand_4(float f);

    private static native long createTonemapDurand_5();

    private static native void dctDenoising_0(long j, long j2, double d, int i);

    private static native void dctDenoising_1(long j, long j2, double d);

    private static native void inpaint_0(long j, long j2, long j3, int i);

    private static native void oilPainting_0(long j, long j2, int i, int i2, int i3);

    private static native void oilPainting_1(long j, long j2, int i, int i2);

    public static GrayworldWB createGrayworldWB() {
        return GrayworldWB.__fromPtr__(createGrayworldWB_0());
    }

    public static LearningBasedWB createLearningBasedWB(String path_to_model) {
        return LearningBasedWB.__fromPtr__(createLearningBasedWB_0(path_to_model));
    }

    public static LearningBasedWB createLearningBasedWB() {
        return LearningBasedWB.__fromPtr__(createLearningBasedWB_1());
    }

    public static SimpleWB createSimpleWB() {
        return SimpleWB.__fromPtr__(createSimpleWB_0());
    }

    public static TonemapDurand createTonemapDurand(float gamma, float contrast, float saturation, float sigma_space, float sigma_color) {
        return TonemapDurand.__fromPtr__(createTonemapDurand_0(gamma, contrast, saturation, sigma_space, sigma_color));
    }

    public static TonemapDurand createTonemapDurand(float gamma, float contrast, float saturation, float sigma_space) {
        return TonemapDurand.__fromPtr__(createTonemapDurand_1(gamma, contrast, saturation, sigma_space));
    }

    public static TonemapDurand createTonemapDurand(float gamma, float contrast, float saturation) {
        return TonemapDurand.__fromPtr__(createTonemapDurand_2(gamma, contrast, saturation));
    }

    public static TonemapDurand createTonemapDurand(float gamma, float contrast) {
        return TonemapDurand.__fromPtr__(createTonemapDurand_3(gamma, contrast));
    }

    public static TonemapDurand createTonemapDurand(float gamma) {
        return TonemapDurand.__fromPtr__(createTonemapDurand_4(gamma));
    }

    public static TonemapDurand createTonemapDurand() {
        return TonemapDurand.__fromPtr__(createTonemapDurand_5());
    }

    public static void applyChannelGains(Mat src, Mat dst, float gainB, float gainG, float gainR) {
        applyChannelGains_0(src.nativeObj, dst.nativeObj, gainB, gainG, gainR);
    }

    public static void bm3dDenoising(Mat src, Mat dst, float h, int templateWindowSize, int searchWindowSize, int blockMatchingStep1, int blockMatchingStep2, int groupSize, int slidingStep, float beta, int normType, int step, int transformType) {
        bm3dDenoising_0(src.nativeObj, dst.nativeObj, h, templateWindowSize, searchWindowSize, blockMatchingStep1, blockMatchingStep2, groupSize, slidingStep, beta, normType, step, transformType);
    }

    public static void bm3dDenoising(Mat src, Mat dst, float h, int templateWindowSize, int searchWindowSize, int blockMatchingStep1, int blockMatchingStep2, int groupSize, int slidingStep, float beta, int normType, int step) {
        bm3dDenoising_1(src.nativeObj, dst.nativeObj, h, templateWindowSize, searchWindowSize, blockMatchingStep1, blockMatchingStep2, groupSize, slidingStep, beta, normType, step);
    }

    public static void bm3dDenoising(Mat src, Mat dst, float h, int templateWindowSize, int searchWindowSize, int blockMatchingStep1, int blockMatchingStep2, int groupSize, int slidingStep, float beta, int normType) {
        bm3dDenoising_2(src.nativeObj, dst.nativeObj, h, templateWindowSize, searchWindowSize, blockMatchingStep1, blockMatchingStep2, groupSize, slidingStep, beta, normType);
    }

    public static void bm3dDenoising(Mat src, Mat dst, float h, int templateWindowSize, int searchWindowSize, int blockMatchingStep1, int blockMatchingStep2, int groupSize, int slidingStep, float beta) {
        bm3dDenoising_3(src.nativeObj, dst.nativeObj, h, templateWindowSize, searchWindowSize, blockMatchingStep1, blockMatchingStep2, groupSize, slidingStep, beta);
    }

    public static void bm3dDenoising(Mat src, Mat dst, float h, int templateWindowSize, int searchWindowSize, int blockMatchingStep1, int blockMatchingStep2, int groupSize, int slidingStep) {
        bm3dDenoising_4(src.nativeObj, dst.nativeObj, h, templateWindowSize, searchWindowSize, blockMatchingStep1, blockMatchingStep2, groupSize, slidingStep);
    }

    public static void bm3dDenoising(Mat src, Mat dst, float h, int templateWindowSize, int searchWindowSize, int blockMatchingStep1, int blockMatchingStep2, int groupSize) {
        bm3dDenoising_5(src.nativeObj, dst.nativeObj, h, templateWindowSize, searchWindowSize, blockMatchingStep1, blockMatchingStep2, groupSize);
    }

    public static void bm3dDenoising(Mat src, Mat dst, float h, int templateWindowSize, int searchWindowSize, int blockMatchingStep1, int blockMatchingStep2) {
        bm3dDenoising_6(src.nativeObj, dst.nativeObj, h, templateWindowSize, searchWindowSize, blockMatchingStep1, blockMatchingStep2);
    }

    public static void bm3dDenoising(Mat src, Mat dst, float h, int templateWindowSize, int searchWindowSize, int blockMatchingStep1) {
        bm3dDenoising_7(src.nativeObj, dst.nativeObj, h, templateWindowSize, searchWindowSize, blockMatchingStep1);
    }

    public static void bm3dDenoising(Mat src, Mat dst, float h, int templateWindowSize, int searchWindowSize) {
        bm3dDenoising_8(src.nativeObj, dst.nativeObj, h, templateWindowSize, searchWindowSize);
    }

    public static void bm3dDenoising(Mat src, Mat dst, float h, int templateWindowSize) {
        bm3dDenoising_9(src.nativeObj, dst.nativeObj, h, templateWindowSize);
    }

    public static void bm3dDenoising(Mat src, Mat dst, float h) {
        bm3dDenoising_10(src.nativeObj, dst.nativeObj, h);
    }

    public static void bm3dDenoising(Mat src, Mat dst) {
        bm3dDenoising_11(src.nativeObj, dst.nativeObj);
    }

    public static void bm3dDenoising(Mat src, Mat dstStep1, Mat dstStep2, float h, int templateWindowSize, int searchWindowSize, int blockMatchingStep1, int blockMatchingStep2, int groupSize, int slidingStep, float beta, int normType, int step, int transformType) {
        long j = src.nativeObj;
        long j2 = j;
        bm3dDenoising_12(j2, dstStep1.nativeObj, dstStep2.nativeObj, h, templateWindowSize, searchWindowSize, blockMatchingStep1, blockMatchingStep2, groupSize, slidingStep, beta, normType, step, transformType);
    }

    public static void bm3dDenoising(Mat src, Mat dstStep1, Mat dstStep2, float h, int templateWindowSize, int searchWindowSize, int blockMatchingStep1, int blockMatchingStep2, int groupSize, int slidingStep, float beta, int normType, int step) {
        bm3dDenoising_13(src.nativeObj, dstStep1.nativeObj, dstStep2.nativeObj, h, templateWindowSize, searchWindowSize, blockMatchingStep1, blockMatchingStep2, groupSize, slidingStep, beta, normType, step);
    }

    public static void bm3dDenoising(Mat src, Mat dstStep1, Mat dstStep2, float h, int templateWindowSize, int searchWindowSize, int blockMatchingStep1, int blockMatchingStep2, int groupSize, int slidingStep, float beta, int normType) {
        bm3dDenoising_14(src.nativeObj, dstStep1.nativeObj, dstStep2.nativeObj, h, templateWindowSize, searchWindowSize, blockMatchingStep1, blockMatchingStep2, groupSize, slidingStep, beta, normType);
    }

    public static void bm3dDenoising(Mat src, Mat dstStep1, Mat dstStep2, float h, int templateWindowSize, int searchWindowSize, int blockMatchingStep1, int blockMatchingStep2, int groupSize, int slidingStep, float beta) {
        bm3dDenoising_15(src.nativeObj, dstStep1.nativeObj, dstStep2.nativeObj, h, templateWindowSize, searchWindowSize, blockMatchingStep1, blockMatchingStep2, groupSize, slidingStep, beta);
    }

    public static void bm3dDenoising(Mat src, Mat dstStep1, Mat dstStep2, float h, int templateWindowSize, int searchWindowSize, int blockMatchingStep1, int blockMatchingStep2, int groupSize, int slidingStep) {
        bm3dDenoising_16(src.nativeObj, dstStep1.nativeObj, dstStep2.nativeObj, h, templateWindowSize, searchWindowSize, blockMatchingStep1, blockMatchingStep2, groupSize, slidingStep);
    }

    public static void bm3dDenoising(Mat src, Mat dstStep1, Mat dstStep2, float h, int templateWindowSize, int searchWindowSize, int blockMatchingStep1, int blockMatchingStep2, int groupSize) {
        bm3dDenoising_17(src.nativeObj, dstStep1.nativeObj, dstStep2.nativeObj, h, templateWindowSize, searchWindowSize, blockMatchingStep1, blockMatchingStep2, groupSize);
    }

    public static void bm3dDenoising(Mat src, Mat dstStep1, Mat dstStep2, float h, int templateWindowSize, int searchWindowSize, int blockMatchingStep1, int blockMatchingStep2) {
        bm3dDenoising_18(src.nativeObj, dstStep1.nativeObj, dstStep2.nativeObj, h, templateWindowSize, searchWindowSize, blockMatchingStep1, blockMatchingStep2);
    }

    public static void bm3dDenoising(Mat src, Mat dstStep1, Mat dstStep2, float h, int templateWindowSize, int searchWindowSize, int blockMatchingStep1) {
        bm3dDenoising_19(src.nativeObj, dstStep1.nativeObj, dstStep2.nativeObj, h, templateWindowSize, searchWindowSize, blockMatchingStep1);
    }

    public static void bm3dDenoising(Mat src, Mat dstStep1, Mat dstStep2, float h, int templateWindowSize, int searchWindowSize) {
        bm3dDenoising_20(src.nativeObj, dstStep1.nativeObj, dstStep2.nativeObj, h, templateWindowSize, searchWindowSize);
    }

    public static void bm3dDenoising(Mat src, Mat dstStep1, Mat dstStep2, float h, int templateWindowSize) {
        bm3dDenoising_21(src.nativeObj, dstStep1.nativeObj, dstStep2.nativeObj, h, templateWindowSize);
    }

    public static void bm3dDenoising(Mat src, Mat dstStep1, Mat dstStep2, float h) {
        bm3dDenoising_22(src.nativeObj, dstStep1.nativeObj, dstStep2.nativeObj, h);
    }

    public static void bm3dDenoising(Mat src, Mat dstStep1, Mat dstStep2) {
        bm3dDenoising_23(src.nativeObj, dstStep1.nativeObj, dstStep2.nativeObj);
    }

    public static void dctDenoising(Mat src, Mat dst, double sigma, int psize) {
        dctDenoising_0(src.nativeObj, dst.nativeObj, sigma, psize);
    }

    public static void dctDenoising(Mat src, Mat dst, double sigma) {
        dctDenoising_1(src.nativeObj, dst.nativeObj, sigma);
    }

    public static void inpaint(Mat src, Mat mask, Mat dst, int algorithmType) {
        inpaint_0(src.nativeObj, mask.nativeObj, dst.nativeObj, algorithmType);
    }

    public static void oilPainting(Mat src, Mat dst, int size, int dynRatio, int code) {
        oilPainting_0(src.nativeObj, dst.nativeObj, size, dynRatio, code);
    }

    public static void oilPainting(Mat src, Mat dst, int size, int dynRatio) {
        oilPainting_1(src.nativeObj, dst.nativeObj, size, dynRatio);
    }
}
