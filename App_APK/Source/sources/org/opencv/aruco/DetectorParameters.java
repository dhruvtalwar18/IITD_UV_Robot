package org.opencv.aruco;

public class DetectorParameters {
    protected final long nativeObj;

    private static native long create_0();

    private static native void delete(long j);

    private static native double get_adaptiveThreshConstant_0(long j);

    private static native int get_adaptiveThreshWinSizeMax_0(long j);

    private static native int get_adaptiveThreshWinSizeMin_0(long j);

    private static native int get_adaptiveThreshWinSizeStep_0(long j);

    private static native float get_aprilTagCriticalRad_0(long j);

    private static native int get_aprilTagDeglitch_0(long j);

    private static native float get_aprilTagMaxLineFitMse_0(long j);

    private static native int get_aprilTagMaxNmaxima_0(long j);

    private static native int get_aprilTagMinClusterPixels_0(long j);

    private static native int get_aprilTagMinWhiteBlackDiff_0(long j);

    private static native float get_aprilTagQuadDecimate_0(long j);

    private static native float get_aprilTagQuadSigma_0(long j);

    private static native int get_cornerRefinementMaxIterations_0(long j);

    private static native int get_cornerRefinementMethod_0(long j);

    private static native double get_cornerRefinementMinAccuracy_0(long j);

    private static native int get_cornerRefinementWinSize_0(long j);

    private static native double get_errorCorrectionRate_0(long j);

    private static native int get_markerBorderBits_0(long j);

    private static native double get_maxErroneousBitsInBorderRate_0(long j);

    private static native double get_maxMarkerPerimeterRate_0(long j);

    private static native double get_minCornerDistanceRate_0(long j);

    private static native int get_minDistanceToBorder_0(long j);

    private static native double get_minMarkerDistanceRate_0(long j);

    private static native double get_minMarkerPerimeterRate_0(long j);

    private static native double get_minOtsuStdDev_0(long j);

    private static native double get_perspectiveRemoveIgnoredMarginPerCell_0(long j);

    private static native int get_perspectiveRemovePixelPerCell_0(long j);

    private static native double get_polygonalApproxAccuracyRate_0(long j);

    private static native void set_adaptiveThreshConstant_0(long j, double d);

    private static native void set_adaptiveThreshWinSizeMax_0(long j, int i);

    private static native void set_adaptiveThreshWinSizeMin_0(long j, int i);

    private static native void set_adaptiveThreshWinSizeStep_0(long j, int i);

    private static native void set_aprilTagCriticalRad_0(long j, float f);

    private static native void set_aprilTagDeglitch_0(long j, int i);

    private static native void set_aprilTagMaxLineFitMse_0(long j, float f);

    private static native void set_aprilTagMaxNmaxima_0(long j, int i);

    private static native void set_aprilTagMinClusterPixels_0(long j, int i);

    private static native void set_aprilTagMinWhiteBlackDiff_0(long j, int i);

    private static native void set_aprilTagQuadDecimate_0(long j, float f);

    private static native void set_aprilTagQuadSigma_0(long j, float f);

    private static native void set_cornerRefinementMaxIterations_0(long j, int i);

    private static native void set_cornerRefinementMethod_0(long j, int i);

    private static native void set_cornerRefinementMinAccuracy_0(long j, double d);

    private static native void set_cornerRefinementWinSize_0(long j, int i);

    private static native void set_errorCorrectionRate_0(long j, double d);

    private static native void set_markerBorderBits_0(long j, int i);

    private static native void set_maxErroneousBitsInBorderRate_0(long j, double d);

    private static native void set_maxMarkerPerimeterRate_0(long j, double d);

    private static native void set_minCornerDistanceRate_0(long j, double d);

    private static native void set_minDistanceToBorder_0(long j, int i);

    private static native void set_minMarkerDistanceRate_0(long j, double d);

    private static native void set_minMarkerPerimeterRate_0(long j, double d);

    private static native void set_minOtsuStdDev_0(long j, double d);

    private static native void set_perspectiveRemoveIgnoredMarginPerCell_0(long j, double d);

    private static native void set_perspectiveRemovePixelPerCell_0(long j, int i);

    private static native void set_polygonalApproxAccuracyRate_0(long j, double d);

    protected DetectorParameters(long addr) {
        this.nativeObj = addr;
    }

    public long getNativeObjAddr() {
        return this.nativeObj;
    }

    public static DetectorParameters __fromPtr__(long addr) {
        return new DetectorParameters(addr);
    }

    public static DetectorParameters create() {
        return __fromPtr__(create_0());
    }

    public int get_adaptiveThreshWinSizeMin() {
        return get_adaptiveThreshWinSizeMin_0(this.nativeObj);
    }

    public void set_adaptiveThreshWinSizeMin(int adaptiveThreshWinSizeMin) {
        set_adaptiveThreshWinSizeMin_0(this.nativeObj, adaptiveThreshWinSizeMin);
    }

    public int get_adaptiveThreshWinSizeMax() {
        return get_adaptiveThreshWinSizeMax_0(this.nativeObj);
    }

    public void set_adaptiveThreshWinSizeMax(int adaptiveThreshWinSizeMax) {
        set_adaptiveThreshWinSizeMax_0(this.nativeObj, adaptiveThreshWinSizeMax);
    }

    public int get_adaptiveThreshWinSizeStep() {
        return get_adaptiveThreshWinSizeStep_0(this.nativeObj);
    }

    public void set_adaptiveThreshWinSizeStep(int adaptiveThreshWinSizeStep) {
        set_adaptiveThreshWinSizeStep_0(this.nativeObj, adaptiveThreshWinSizeStep);
    }

    public double get_adaptiveThreshConstant() {
        return get_adaptiveThreshConstant_0(this.nativeObj);
    }

    public void set_adaptiveThreshConstant(double adaptiveThreshConstant) {
        set_adaptiveThreshConstant_0(this.nativeObj, adaptiveThreshConstant);
    }

    public double get_minMarkerPerimeterRate() {
        return get_minMarkerPerimeterRate_0(this.nativeObj);
    }

    public void set_minMarkerPerimeterRate(double minMarkerPerimeterRate) {
        set_minMarkerPerimeterRate_0(this.nativeObj, minMarkerPerimeterRate);
    }

    public double get_maxMarkerPerimeterRate() {
        return get_maxMarkerPerimeterRate_0(this.nativeObj);
    }

    public void set_maxMarkerPerimeterRate(double maxMarkerPerimeterRate) {
        set_maxMarkerPerimeterRate_0(this.nativeObj, maxMarkerPerimeterRate);
    }

    public double get_polygonalApproxAccuracyRate() {
        return get_polygonalApproxAccuracyRate_0(this.nativeObj);
    }

    public void set_polygonalApproxAccuracyRate(double polygonalApproxAccuracyRate) {
        set_polygonalApproxAccuracyRate_0(this.nativeObj, polygonalApproxAccuracyRate);
    }

    public double get_minCornerDistanceRate() {
        return get_minCornerDistanceRate_0(this.nativeObj);
    }

    public void set_minCornerDistanceRate(double minCornerDistanceRate) {
        set_minCornerDistanceRate_0(this.nativeObj, minCornerDistanceRate);
    }

    public int get_minDistanceToBorder() {
        return get_minDistanceToBorder_0(this.nativeObj);
    }

    public void set_minDistanceToBorder(int minDistanceToBorder) {
        set_minDistanceToBorder_0(this.nativeObj, minDistanceToBorder);
    }

    public double get_minMarkerDistanceRate() {
        return get_minMarkerDistanceRate_0(this.nativeObj);
    }

    public void set_minMarkerDistanceRate(double minMarkerDistanceRate) {
        set_minMarkerDistanceRate_0(this.nativeObj, minMarkerDistanceRate);
    }

    public int get_cornerRefinementMethod() {
        return get_cornerRefinementMethod_0(this.nativeObj);
    }

    public void set_cornerRefinementMethod(int cornerRefinementMethod) {
        set_cornerRefinementMethod_0(this.nativeObj, cornerRefinementMethod);
    }

    public int get_cornerRefinementWinSize() {
        return get_cornerRefinementWinSize_0(this.nativeObj);
    }

    public void set_cornerRefinementWinSize(int cornerRefinementWinSize) {
        set_cornerRefinementWinSize_0(this.nativeObj, cornerRefinementWinSize);
    }

    public int get_cornerRefinementMaxIterations() {
        return get_cornerRefinementMaxIterations_0(this.nativeObj);
    }

    public void set_cornerRefinementMaxIterations(int cornerRefinementMaxIterations) {
        set_cornerRefinementMaxIterations_0(this.nativeObj, cornerRefinementMaxIterations);
    }

    public double get_cornerRefinementMinAccuracy() {
        return get_cornerRefinementMinAccuracy_0(this.nativeObj);
    }

    public void set_cornerRefinementMinAccuracy(double cornerRefinementMinAccuracy) {
        set_cornerRefinementMinAccuracy_0(this.nativeObj, cornerRefinementMinAccuracy);
    }

    public int get_markerBorderBits() {
        return get_markerBorderBits_0(this.nativeObj);
    }

    public void set_markerBorderBits(int markerBorderBits) {
        set_markerBorderBits_0(this.nativeObj, markerBorderBits);
    }

    public int get_perspectiveRemovePixelPerCell() {
        return get_perspectiveRemovePixelPerCell_0(this.nativeObj);
    }

    public void set_perspectiveRemovePixelPerCell(int perspectiveRemovePixelPerCell) {
        set_perspectiveRemovePixelPerCell_0(this.nativeObj, perspectiveRemovePixelPerCell);
    }

    public double get_perspectiveRemoveIgnoredMarginPerCell() {
        return get_perspectiveRemoveIgnoredMarginPerCell_0(this.nativeObj);
    }

    public void set_perspectiveRemoveIgnoredMarginPerCell(double perspectiveRemoveIgnoredMarginPerCell) {
        set_perspectiveRemoveIgnoredMarginPerCell_0(this.nativeObj, perspectiveRemoveIgnoredMarginPerCell);
    }

    public double get_maxErroneousBitsInBorderRate() {
        return get_maxErroneousBitsInBorderRate_0(this.nativeObj);
    }

    public void set_maxErroneousBitsInBorderRate(double maxErroneousBitsInBorderRate) {
        set_maxErroneousBitsInBorderRate_0(this.nativeObj, maxErroneousBitsInBorderRate);
    }

    public double get_minOtsuStdDev() {
        return get_minOtsuStdDev_0(this.nativeObj);
    }

    public void set_minOtsuStdDev(double minOtsuStdDev) {
        set_minOtsuStdDev_0(this.nativeObj, minOtsuStdDev);
    }

    public double get_errorCorrectionRate() {
        return get_errorCorrectionRate_0(this.nativeObj);
    }

    public void set_errorCorrectionRate(double errorCorrectionRate) {
        set_errorCorrectionRate_0(this.nativeObj, errorCorrectionRate);
    }

    public float get_aprilTagQuadDecimate() {
        return get_aprilTagQuadDecimate_0(this.nativeObj);
    }

    public void set_aprilTagQuadDecimate(float aprilTagQuadDecimate) {
        set_aprilTagQuadDecimate_0(this.nativeObj, aprilTagQuadDecimate);
    }

    public float get_aprilTagQuadSigma() {
        return get_aprilTagQuadSigma_0(this.nativeObj);
    }

    public void set_aprilTagQuadSigma(float aprilTagQuadSigma) {
        set_aprilTagQuadSigma_0(this.nativeObj, aprilTagQuadSigma);
    }

    public int get_aprilTagMinClusterPixels() {
        return get_aprilTagMinClusterPixels_0(this.nativeObj);
    }

    public void set_aprilTagMinClusterPixels(int aprilTagMinClusterPixels) {
        set_aprilTagMinClusterPixels_0(this.nativeObj, aprilTagMinClusterPixels);
    }

    public int get_aprilTagMaxNmaxima() {
        return get_aprilTagMaxNmaxima_0(this.nativeObj);
    }

    public void set_aprilTagMaxNmaxima(int aprilTagMaxNmaxima) {
        set_aprilTagMaxNmaxima_0(this.nativeObj, aprilTagMaxNmaxima);
    }

    public float get_aprilTagCriticalRad() {
        return get_aprilTagCriticalRad_0(this.nativeObj);
    }

    public void set_aprilTagCriticalRad(float aprilTagCriticalRad) {
        set_aprilTagCriticalRad_0(this.nativeObj, aprilTagCriticalRad);
    }

    public float get_aprilTagMaxLineFitMse() {
        return get_aprilTagMaxLineFitMse_0(this.nativeObj);
    }

    public void set_aprilTagMaxLineFitMse(float aprilTagMaxLineFitMse) {
        set_aprilTagMaxLineFitMse_0(this.nativeObj, aprilTagMaxLineFitMse);
    }

    public int get_aprilTagMinWhiteBlackDiff() {
        return get_aprilTagMinWhiteBlackDiff_0(this.nativeObj);
    }

    public void set_aprilTagMinWhiteBlackDiff(int aprilTagMinWhiteBlackDiff) {
        set_aprilTagMinWhiteBlackDiff_0(this.nativeObj, aprilTagMinWhiteBlackDiff);
    }

    public int get_aprilTagDeglitch() {
        return get_aprilTagDeglitch_0(this.nativeObj);
    }

    public void set_aprilTagDeglitch(int aprilTagDeglitch) {
        set_aprilTagDeglitch_0(this.nativeObj, aprilTagDeglitch);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
