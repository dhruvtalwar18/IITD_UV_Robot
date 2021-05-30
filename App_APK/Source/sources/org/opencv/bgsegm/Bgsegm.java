package org.opencv.bgsegm;

import org.opencv.core.Mat;

public class Bgsegm {
    public static final int LSBP_CAMERA_MOTION_COMPENSATION_LK = 1;
    public static final int LSBP_CAMERA_MOTION_COMPENSATION_NONE = 0;

    private static native long createBackgroundSubtractorCNT_0(int i, boolean z, int i2, boolean z2);

    private static native long createBackgroundSubtractorCNT_1(int i, boolean z, int i2);

    private static native long createBackgroundSubtractorCNT_2(int i, boolean z);

    private static native long createBackgroundSubtractorCNT_3(int i);

    private static native long createBackgroundSubtractorCNT_4();

    private static native long createBackgroundSubtractorGMG_0(int i, double d);

    private static native long createBackgroundSubtractorGMG_1(int i);

    private static native long createBackgroundSubtractorGMG_2();

    private static native long createBackgroundSubtractorGSOC_0(int i, int i2, float f, float f2, int i3, float f3, float f4, float f5, float f6, float f7, float f8);

    private static native long createBackgroundSubtractorGSOC_1(int i, int i2, float f, float f2, int i3, float f3, float f4, float f5, float f6, float f7);

    private static native long createBackgroundSubtractorGSOC_10(int i);

    private static native long createBackgroundSubtractorGSOC_11();

    private static native long createBackgroundSubtractorGSOC_2(int i, int i2, float f, float f2, int i3, float f3, float f4, float f5, float f6);

    private static native long createBackgroundSubtractorGSOC_3(int i, int i2, float f, float f2, int i3, float f3, float f4, float f5);

    private static native long createBackgroundSubtractorGSOC_4(int i, int i2, float f, float f2, int i3, float f3, float f4);

    private static native long createBackgroundSubtractorGSOC_5(int i, int i2, float f, float f2, int i3, float f3);

    private static native long createBackgroundSubtractorGSOC_6(int i, int i2, float f, float f2, int i3);

    private static native long createBackgroundSubtractorGSOC_7(int i, int i2, float f, float f2);

    private static native long createBackgroundSubtractorGSOC_8(int i, int i2, float f);

    private static native long createBackgroundSubtractorGSOC_9(int i, int i2);

    private static native long createBackgroundSubtractorLSBP_0(int i, int i2, int i3, float f, float f2, float f3, float f4, float f5, float f6, float f7, float f8, int i4, int i5);

    private static native long createBackgroundSubtractorLSBP_1(int i, int i2, int i3, float f, float f2, float f3, float f4, float f5, float f6, float f7, float f8, int i4);

    private static native long createBackgroundSubtractorLSBP_10(int i, int i2, int i3);

    private static native long createBackgroundSubtractorLSBP_11(int i, int i2);

    private static native long createBackgroundSubtractorLSBP_12(int i);

    private static native long createBackgroundSubtractorLSBP_13();

    private static native long createBackgroundSubtractorLSBP_2(int i, int i2, int i3, float f, float f2, float f3, float f4, float f5, float f6, float f7, float f8);

    private static native long createBackgroundSubtractorLSBP_3(int i, int i2, int i3, float f, float f2, float f3, float f4, float f5, float f6, float f7);

    private static native long createBackgroundSubtractorLSBP_4(int i, int i2, int i3, float f, float f2, float f3, float f4, float f5, float f6);

    private static native long createBackgroundSubtractorLSBP_5(int i, int i2, int i3, float f, float f2, float f3, float f4, float f5);

    private static native long createBackgroundSubtractorLSBP_6(int i, int i2, int i3, float f, float f2, float f3, float f4);

    private static native long createBackgroundSubtractorLSBP_7(int i, int i2, int i3, float f, float f2, float f3);

    private static native long createBackgroundSubtractorLSBP_8(int i, int i2, int i3, float f, float f2);

    private static native long createBackgroundSubtractorLSBP_9(int i, int i2, int i3, float f);

    private static native long createBackgroundSubtractorMOG_0(int i, int i2, double d, double d2);

    private static native long createBackgroundSubtractorMOG_1(int i, int i2, double d);

    private static native long createBackgroundSubtractorMOG_2(int i, int i2);

    private static native long createBackgroundSubtractorMOG_3(int i);

    private static native long createBackgroundSubtractorMOG_4();

    private static native long createSyntheticSequenceGenerator_0(long j, long j2, double d, double d2, double d3, double d4);

    private static native long createSyntheticSequenceGenerator_1(long j, long j2, double d, double d2, double d3);

    private static native long createSyntheticSequenceGenerator_2(long j, long j2, double d, double d2);

    private static native long createSyntheticSequenceGenerator_3(long j, long j2, double d);

    private static native long createSyntheticSequenceGenerator_4(long j, long j2);

    public static BackgroundSubtractorCNT createBackgroundSubtractorCNT(int minPixelStability, boolean useHistory, int maxPixelStability, boolean isParallel) {
        return BackgroundSubtractorCNT.__fromPtr__(createBackgroundSubtractorCNT_0(minPixelStability, useHistory, maxPixelStability, isParallel));
    }

    public static BackgroundSubtractorCNT createBackgroundSubtractorCNT(int minPixelStability, boolean useHistory, int maxPixelStability) {
        return BackgroundSubtractorCNT.__fromPtr__(createBackgroundSubtractorCNT_1(minPixelStability, useHistory, maxPixelStability));
    }

    public static BackgroundSubtractorCNT createBackgroundSubtractorCNT(int minPixelStability, boolean useHistory) {
        return BackgroundSubtractorCNT.__fromPtr__(createBackgroundSubtractorCNT_2(minPixelStability, useHistory));
    }

    public static BackgroundSubtractorCNT createBackgroundSubtractorCNT(int minPixelStability) {
        return BackgroundSubtractorCNT.__fromPtr__(createBackgroundSubtractorCNT_3(minPixelStability));
    }

    public static BackgroundSubtractorCNT createBackgroundSubtractorCNT() {
        return BackgroundSubtractorCNT.__fromPtr__(createBackgroundSubtractorCNT_4());
    }

    public static BackgroundSubtractorGMG createBackgroundSubtractorGMG(int initializationFrames, double decisionThreshold) {
        return BackgroundSubtractorGMG.__fromPtr__(createBackgroundSubtractorGMG_0(initializationFrames, decisionThreshold));
    }

    public static BackgroundSubtractorGMG createBackgroundSubtractorGMG(int initializationFrames) {
        return BackgroundSubtractorGMG.__fromPtr__(createBackgroundSubtractorGMG_1(initializationFrames));
    }

    public static BackgroundSubtractorGMG createBackgroundSubtractorGMG() {
        return BackgroundSubtractorGMG.__fromPtr__(createBackgroundSubtractorGMG_2());
    }

    public static BackgroundSubtractorGSOC createBackgroundSubtractorGSOC(int mc, int nSamples, float replaceRate, float propagationRate, int hitsThreshold, float alpha, float beta, float blinkingSupressionDecay, float blinkingSupressionMultiplier, float noiseRemovalThresholdFacBG, float noiseRemovalThresholdFacFG) {
        return BackgroundSubtractorGSOC.__fromPtr__(createBackgroundSubtractorGSOC_0(mc, nSamples, replaceRate, propagationRate, hitsThreshold, alpha, beta, blinkingSupressionDecay, blinkingSupressionMultiplier, noiseRemovalThresholdFacBG, noiseRemovalThresholdFacFG));
    }

    public static BackgroundSubtractorGSOC createBackgroundSubtractorGSOC(int mc, int nSamples, float replaceRate, float propagationRate, int hitsThreshold, float alpha, float beta, float blinkingSupressionDecay, float blinkingSupressionMultiplier, float noiseRemovalThresholdFacBG) {
        return BackgroundSubtractorGSOC.__fromPtr__(createBackgroundSubtractorGSOC_1(mc, nSamples, replaceRate, propagationRate, hitsThreshold, alpha, beta, blinkingSupressionDecay, blinkingSupressionMultiplier, noiseRemovalThresholdFacBG));
    }

    public static BackgroundSubtractorGSOC createBackgroundSubtractorGSOC(int mc, int nSamples, float replaceRate, float propagationRate, int hitsThreshold, float alpha, float beta, float blinkingSupressionDecay, float blinkingSupressionMultiplier) {
        return BackgroundSubtractorGSOC.__fromPtr__(createBackgroundSubtractorGSOC_2(mc, nSamples, replaceRate, propagationRate, hitsThreshold, alpha, beta, blinkingSupressionDecay, blinkingSupressionMultiplier));
    }

    public static BackgroundSubtractorGSOC createBackgroundSubtractorGSOC(int mc, int nSamples, float replaceRate, float propagationRate, int hitsThreshold, float alpha, float beta, float blinkingSupressionDecay) {
        return BackgroundSubtractorGSOC.__fromPtr__(createBackgroundSubtractorGSOC_3(mc, nSamples, replaceRate, propagationRate, hitsThreshold, alpha, beta, blinkingSupressionDecay));
    }

    public static BackgroundSubtractorGSOC createBackgroundSubtractorGSOC(int mc, int nSamples, float replaceRate, float propagationRate, int hitsThreshold, float alpha, float beta) {
        return BackgroundSubtractorGSOC.__fromPtr__(createBackgroundSubtractorGSOC_4(mc, nSamples, replaceRate, propagationRate, hitsThreshold, alpha, beta));
    }

    public static BackgroundSubtractorGSOC createBackgroundSubtractorGSOC(int mc, int nSamples, float replaceRate, float propagationRate, int hitsThreshold, float alpha) {
        return BackgroundSubtractorGSOC.__fromPtr__(createBackgroundSubtractorGSOC_5(mc, nSamples, replaceRate, propagationRate, hitsThreshold, alpha));
    }

    public static BackgroundSubtractorGSOC createBackgroundSubtractorGSOC(int mc, int nSamples, float replaceRate, float propagationRate, int hitsThreshold) {
        return BackgroundSubtractorGSOC.__fromPtr__(createBackgroundSubtractorGSOC_6(mc, nSamples, replaceRate, propagationRate, hitsThreshold));
    }

    public static BackgroundSubtractorGSOC createBackgroundSubtractorGSOC(int mc, int nSamples, float replaceRate, float propagationRate) {
        return BackgroundSubtractorGSOC.__fromPtr__(createBackgroundSubtractorGSOC_7(mc, nSamples, replaceRate, propagationRate));
    }

    public static BackgroundSubtractorGSOC createBackgroundSubtractorGSOC(int mc, int nSamples, float replaceRate) {
        return BackgroundSubtractorGSOC.__fromPtr__(createBackgroundSubtractorGSOC_8(mc, nSamples, replaceRate));
    }

    public static BackgroundSubtractorGSOC createBackgroundSubtractorGSOC(int mc, int nSamples) {
        return BackgroundSubtractorGSOC.__fromPtr__(createBackgroundSubtractorGSOC_9(mc, nSamples));
    }

    public static BackgroundSubtractorGSOC createBackgroundSubtractorGSOC(int mc) {
        return BackgroundSubtractorGSOC.__fromPtr__(createBackgroundSubtractorGSOC_10(mc));
    }

    public static BackgroundSubtractorGSOC createBackgroundSubtractorGSOC() {
        return BackgroundSubtractorGSOC.__fromPtr__(createBackgroundSubtractorGSOC_11());
    }

    public static BackgroundSubtractorLSBP createBackgroundSubtractorLSBP(int mc, int nSamples, int LSBPRadius, float Tlower, float Tupper, float Tinc, float Tdec, float Rscale, float Rincdec, float noiseRemovalThresholdFacBG, float noiseRemovalThresholdFacFG, int LSBPthreshold, int minCount) {
        return BackgroundSubtractorLSBP.__fromPtr__(createBackgroundSubtractorLSBP_0(mc, nSamples, LSBPRadius, Tlower, Tupper, Tinc, Tdec, Rscale, Rincdec, noiseRemovalThresholdFacBG, noiseRemovalThresholdFacFG, LSBPthreshold, minCount));
    }

    public static BackgroundSubtractorLSBP createBackgroundSubtractorLSBP(int mc, int nSamples, int LSBPRadius, float Tlower, float Tupper, float Tinc, float Tdec, float Rscale, float Rincdec, float noiseRemovalThresholdFacBG, float noiseRemovalThresholdFacFG, int LSBPthreshold) {
        return BackgroundSubtractorLSBP.__fromPtr__(createBackgroundSubtractorLSBP_1(mc, nSamples, LSBPRadius, Tlower, Tupper, Tinc, Tdec, Rscale, Rincdec, noiseRemovalThresholdFacBG, noiseRemovalThresholdFacFG, LSBPthreshold));
    }

    public static BackgroundSubtractorLSBP createBackgroundSubtractorLSBP(int mc, int nSamples, int LSBPRadius, float Tlower, float Tupper, float Tinc, float Tdec, float Rscale, float Rincdec, float noiseRemovalThresholdFacBG, float noiseRemovalThresholdFacFG) {
        return BackgroundSubtractorLSBP.__fromPtr__(createBackgroundSubtractorLSBP_2(mc, nSamples, LSBPRadius, Tlower, Tupper, Tinc, Tdec, Rscale, Rincdec, noiseRemovalThresholdFacBG, noiseRemovalThresholdFacFG));
    }

    public static BackgroundSubtractorLSBP createBackgroundSubtractorLSBP(int mc, int nSamples, int LSBPRadius, float Tlower, float Tupper, float Tinc, float Tdec, float Rscale, float Rincdec, float noiseRemovalThresholdFacBG) {
        return BackgroundSubtractorLSBP.__fromPtr__(createBackgroundSubtractorLSBP_3(mc, nSamples, LSBPRadius, Tlower, Tupper, Tinc, Tdec, Rscale, Rincdec, noiseRemovalThresholdFacBG));
    }

    public static BackgroundSubtractorLSBP createBackgroundSubtractorLSBP(int mc, int nSamples, int LSBPRadius, float Tlower, float Tupper, float Tinc, float Tdec, float Rscale, float Rincdec) {
        return BackgroundSubtractorLSBP.__fromPtr__(createBackgroundSubtractorLSBP_4(mc, nSamples, LSBPRadius, Tlower, Tupper, Tinc, Tdec, Rscale, Rincdec));
    }

    public static BackgroundSubtractorLSBP createBackgroundSubtractorLSBP(int mc, int nSamples, int LSBPRadius, float Tlower, float Tupper, float Tinc, float Tdec, float Rscale) {
        return BackgroundSubtractorLSBP.__fromPtr__(createBackgroundSubtractorLSBP_5(mc, nSamples, LSBPRadius, Tlower, Tupper, Tinc, Tdec, Rscale));
    }

    public static BackgroundSubtractorLSBP createBackgroundSubtractorLSBP(int mc, int nSamples, int LSBPRadius, float Tlower, float Tupper, float Tinc, float Tdec) {
        return BackgroundSubtractorLSBP.__fromPtr__(createBackgroundSubtractorLSBP_6(mc, nSamples, LSBPRadius, Tlower, Tupper, Tinc, Tdec));
    }

    public static BackgroundSubtractorLSBP createBackgroundSubtractorLSBP(int mc, int nSamples, int LSBPRadius, float Tlower, float Tupper, float Tinc) {
        return BackgroundSubtractorLSBP.__fromPtr__(createBackgroundSubtractorLSBP_7(mc, nSamples, LSBPRadius, Tlower, Tupper, Tinc));
    }

    public static BackgroundSubtractorLSBP createBackgroundSubtractorLSBP(int mc, int nSamples, int LSBPRadius, float Tlower, float Tupper) {
        return BackgroundSubtractorLSBP.__fromPtr__(createBackgroundSubtractorLSBP_8(mc, nSamples, LSBPRadius, Tlower, Tupper));
    }

    public static BackgroundSubtractorLSBP createBackgroundSubtractorLSBP(int mc, int nSamples, int LSBPRadius, float Tlower) {
        return BackgroundSubtractorLSBP.__fromPtr__(createBackgroundSubtractorLSBP_9(mc, nSamples, LSBPRadius, Tlower));
    }

    public static BackgroundSubtractorLSBP createBackgroundSubtractorLSBP(int mc, int nSamples, int LSBPRadius) {
        return BackgroundSubtractorLSBP.__fromPtr__(createBackgroundSubtractorLSBP_10(mc, nSamples, LSBPRadius));
    }

    public static BackgroundSubtractorLSBP createBackgroundSubtractorLSBP(int mc, int nSamples) {
        return BackgroundSubtractorLSBP.__fromPtr__(createBackgroundSubtractorLSBP_11(mc, nSamples));
    }

    public static BackgroundSubtractorLSBP createBackgroundSubtractorLSBP(int mc) {
        return BackgroundSubtractorLSBP.__fromPtr__(createBackgroundSubtractorLSBP_12(mc));
    }

    public static BackgroundSubtractorLSBP createBackgroundSubtractorLSBP() {
        return BackgroundSubtractorLSBP.__fromPtr__(createBackgroundSubtractorLSBP_13());
    }

    public static BackgroundSubtractorMOG createBackgroundSubtractorMOG(int history, int nmixtures, double backgroundRatio, double noiseSigma) {
        return BackgroundSubtractorMOG.__fromPtr__(createBackgroundSubtractorMOG_0(history, nmixtures, backgroundRatio, noiseSigma));
    }

    public static BackgroundSubtractorMOG createBackgroundSubtractorMOG(int history, int nmixtures, double backgroundRatio) {
        return BackgroundSubtractorMOG.__fromPtr__(createBackgroundSubtractorMOG_1(history, nmixtures, backgroundRatio));
    }

    public static BackgroundSubtractorMOG createBackgroundSubtractorMOG(int history, int nmixtures) {
        return BackgroundSubtractorMOG.__fromPtr__(createBackgroundSubtractorMOG_2(history, nmixtures));
    }

    public static BackgroundSubtractorMOG createBackgroundSubtractorMOG(int history) {
        return BackgroundSubtractorMOG.__fromPtr__(createBackgroundSubtractorMOG_3(history));
    }

    public static BackgroundSubtractorMOG createBackgroundSubtractorMOG() {
        return BackgroundSubtractorMOG.__fromPtr__(createBackgroundSubtractorMOG_4());
    }

    public static SyntheticSequenceGenerator createSyntheticSequenceGenerator(Mat background, Mat object, double amplitude, double wavelength, double wavespeed, double objspeed) {
        return SyntheticSequenceGenerator.__fromPtr__(createSyntheticSequenceGenerator_0(background.nativeObj, object.nativeObj, amplitude, wavelength, wavespeed, objspeed));
    }

    public static SyntheticSequenceGenerator createSyntheticSequenceGenerator(Mat background, Mat object, double amplitude, double wavelength, double wavespeed) {
        return SyntheticSequenceGenerator.__fromPtr__(createSyntheticSequenceGenerator_1(background.nativeObj, object.nativeObj, amplitude, wavelength, wavespeed));
    }

    public static SyntheticSequenceGenerator createSyntheticSequenceGenerator(Mat background, Mat object, double amplitude, double wavelength) {
        return SyntheticSequenceGenerator.__fromPtr__(createSyntheticSequenceGenerator_2(background.nativeObj, object.nativeObj, amplitude, wavelength));
    }

    public static SyntheticSequenceGenerator createSyntheticSequenceGenerator(Mat background, Mat object, double amplitude) {
        return SyntheticSequenceGenerator.__fromPtr__(createSyntheticSequenceGenerator_3(background.nativeObj, object.nativeObj, amplitude));
    }

    public static SyntheticSequenceGenerator createSyntheticSequenceGenerator(Mat background, Mat object) {
        return SyntheticSequenceGenerator.__fromPtr__(createSyntheticSequenceGenerator_4(background.nativeObj, object.nativeObj));
    }
}
