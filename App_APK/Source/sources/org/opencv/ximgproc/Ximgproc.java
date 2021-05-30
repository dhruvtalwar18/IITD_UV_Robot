package org.opencv.ximgproc;

import org.opencv.calib3d.StereoMatcher;
import org.opencv.core.Mat;

public class Ximgproc {
    public static final int AM_FILTER = 4;
    public static final int ARO_0_45 = 0;
    public static final int ARO_315_0 = 3;
    public static final int ARO_315_135 = 6;
    public static final int ARO_315_45 = 4;
    public static final int ARO_45_135 = 5;
    public static final int ARO_45_90 = 1;
    public static final int ARO_90_135 = 2;
    public static final int ARO_CTR_HOR = 7;
    public static final int ARO_CTR_VER = 8;
    public static final int BINARIZATION_NIBLACK = 0;
    public static final int BINARIZATION_NICK = 3;
    public static final int BINARIZATION_SAUVOLA = 1;
    public static final int BINARIZATION_WOLF = 2;
    public static final int DTF_IC = 1;
    public static final int DTF_NC = 0;
    public static final int DTF_RF = 2;
    public static final int FHT_ADD = 2;
    public static final int FHT_AVE = 3;
    public static final int FHT_MAX = 1;
    public static final int FHT_MIN = 0;
    public static final int GUIDED_FILTER = 3;
    public static final int HDO_DESKEW = 1;
    public static final int HDO_RAW = 0;
    public static final int MSLIC = 102;
    public static final int SLIC = 100;
    public static final int SLICO = 101;
    public static final int THINNING_GUOHALL = 1;
    public static final int THINNING_ZHANGSUEN = 0;
    public static final int WMF_COS = 8;
    public static final int WMF_EXP = 1;
    public static final int WMF_IV1 = 2;
    public static final int WMF_IV2 = 4;
    public static final int WMF_JAC = 16;
    public static final int WMF_OFF = 32;

    private static native void FastHoughTransform_0(long j, long j2, int i, int i2, int i3, int i4);

    private static native void FastHoughTransform_1(long j, long j2, int i, int i2, int i3);

    private static native void FastHoughTransform_2(long j, long j2, int i, int i2);

    private static native void FastHoughTransform_3(long j, long j2, int i);

    private static native void GradientDericheX_0(long j, long j2, double d, double d2);

    private static native void GradientDericheY_0(long j, long j2, double d, double d2);

    private static native void PeiLinNormalization_0(long j, long j2);

    private static native void amFilter_0(long j, long j2, long j3, double d, double d2, boolean z);

    private static native void amFilter_1(long j, long j2, long j3, double d, double d2);

    private static native void anisotropicDiffusion_0(long j, long j2, float f, float f2, int i);

    private static native void bilateralTextureFilter_0(long j, long j2, int i, int i2, double d, double d2);

    private static native void bilateralTextureFilter_1(long j, long j2, int i, int i2, double d);

    private static native void bilateralTextureFilter_2(long j, long j2, int i, int i2);

    private static native void bilateralTextureFilter_3(long j, long j2, int i);

    private static native void bilateralTextureFilter_4(long j, long j2);

    private static native void colorMatchTemplate_0(long j, long j2, long j3);

    private static native void contourSampling_0(long j, long j2, int i);

    private static native void covarianceEstimation_0(long j, long j2, int i, int i2);

    private static native long createAMFilter_0(double d, double d2, boolean z);

    private static native long createAMFilter_1(double d, double d2);

    private static native long createContourFitting_0(int i, int i2);

    private static native long createContourFitting_1(int i);

    private static native long createContourFitting_2();

    private static native long createDTFilter_0(long j, double d, double d2, int i, int i2);

    private static native long createDTFilter_1(long j, double d, double d2, int i);

    private static native long createDTFilter_2(long j, double d, double d2);

    private static native long createDisparityWLSFilterGeneric_0(boolean z);

    private static native long createDisparityWLSFilter_0(long j);

    private static native long createEdgeAwareInterpolator_0();

    private static native long createEdgeBoxes_0(float f, float f2, float f3, float f4, int i, float f5, float f6, float f7, float f8, float f9, float f10, float f11);

    private static native long createEdgeBoxes_1(float f, float f2, float f3, float f4, int i, float f5, float f6, float f7, float f8, float f9, float f10);

    private static native long createEdgeBoxes_10(float f, float f2);

    private static native long createEdgeBoxes_11(float f);

    private static native long createEdgeBoxes_12();

    private static native long createEdgeBoxes_2(float f, float f2, float f3, float f4, int i, float f5, float f6, float f7, float f8, float f9);

    private static native long createEdgeBoxes_3(float f, float f2, float f3, float f4, int i, float f5, float f6, float f7, float f8);

    private static native long createEdgeBoxes_4(float f, float f2, float f3, float f4, int i, float f5, float f6, float f7);

    private static native long createEdgeBoxes_5(float f, float f2, float f3, float f4, int i, float f5, float f6);

    private static native long createEdgeBoxes_6(float f, float f2, float f3, float f4, int i, float f5);

    private static native long createEdgeBoxes_7(float f, float f2, float f3, float f4, int i);

    private static native long createEdgeBoxes_8(float f, float f2, float f3, float f4);

    private static native long createEdgeBoxes_9(float f, float f2, float f3);

    private static native long createFastBilateralSolverFilter_0(long j, double d, double d2, double d3, double d4, int i, double d5);

    private static native long createFastBilateralSolverFilter_1(long j, double d, double d2, double d3, double d4, int i);

    private static native long createFastBilateralSolverFilter_2(long j, double d, double d2, double d3, double d4);

    private static native long createFastBilateralSolverFilter_3(long j, double d, double d2, double d3);

    private static native long createFastGlobalSmootherFilter_0(long j, double d, double d2, double d3, int i);

    private static native long createFastGlobalSmootherFilter_1(long j, double d, double d2, double d3);

    private static native long createFastGlobalSmootherFilter_2(long j, double d, double d2);

    private static native long createFastLineDetector_0(int i, float f, double d, double d2, int i2, boolean z);

    private static native long createFastLineDetector_1(int i, float f, double d, double d2, int i2);

    private static native long createFastLineDetector_2(int i, float f, double d, double d2);

    private static native long createFastLineDetector_3(int i, float f, double d);

    private static native long createFastLineDetector_4(int i, float f);

    private static native long createFastLineDetector_5(int i);

    private static native long createFastLineDetector_6();

    private static native long createGraphSegmentation_0(double d, float f, int i);

    private static native long createGraphSegmentation_1(double d, float f);

    private static native long createGraphSegmentation_2(double d);

    private static native long createGraphSegmentation_3();

    private static native long createGuidedFilter_0(long j, int i, double d);

    private static native void createQuaternionImage_0(long j, long j2);

    private static native long createRFFeatureGetter_0();

    private static native long createRightMatcher_0(long j);

    private static native long createSelectiveSearchSegmentationStrategyColor_0();

    private static native long createSelectiveSearchSegmentationStrategyFill_0();

    private static native long createSelectiveSearchSegmentationStrategyMultiple_0(long j, long j2, long j3, long j4);

    private static native long createSelectiveSearchSegmentationStrategyMultiple_1(long j, long j2, long j3);

    private static native long createSelectiveSearchSegmentationStrategyMultiple_2(long j, long j2);

    private static native long createSelectiveSearchSegmentationStrategyMultiple_3(long j);

    private static native long createSelectiveSearchSegmentationStrategyMultiple_4();

    private static native long createSelectiveSearchSegmentationStrategySize_0();

    private static native long createSelectiveSearchSegmentationStrategyTexture_0();

    private static native long createSelectiveSearchSegmentation_0();

    private static native long createStructuredEdgeDetection_0(String str, long j);

    private static native long createStructuredEdgeDetection_1(String str);

    private static native long createSuperpixelLSC_0(long j, int i, float f);

    private static native long createSuperpixelLSC_1(long j, int i);

    private static native long createSuperpixelLSC_2(long j);

    private static native long createSuperpixelSEEDS_0(int i, int i2, int i3, int i4, int i5, int i6, int i7, boolean z);

    private static native long createSuperpixelSEEDS_1(int i, int i2, int i3, int i4, int i5, int i6, int i7);

    private static native long createSuperpixelSEEDS_2(int i, int i2, int i3, int i4, int i5, int i6);

    private static native long createSuperpixelSEEDS_3(int i, int i2, int i3, int i4, int i5);

    private static native long createSuperpixelSLIC_0(long j, int i, int i2, float f);

    private static native long createSuperpixelSLIC_1(long j, int i, int i2);

    private static native long createSuperpixelSLIC_2(long j, int i);

    private static native long createSuperpixelSLIC_3(long j);

    private static native void dtFilter_0(long j, long j2, long j3, double d, double d2, int i, int i2);

    private static native void dtFilter_1(long j, long j2, long j3, double d, double d2, int i);

    private static native void dtFilter_2(long j, long j2, long j3, double d, double d2);

    private static native void edgePreservingFilter_0(long j, long j2, int i, double d);

    private static native void fastBilateralSolverFilter_0(long j, long j2, long j3, long j4, double d, double d2, double d3, double d4, int i, double d5);

    private static native void fastBilateralSolverFilter_1(long j, long j2, long j3, long j4, double d, double d2, double d3, double d4, int i);

    private static native void fastBilateralSolverFilter_2(long j, long j2, long j3, long j4, double d, double d2, double d3, double d4);

    private static native void fastBilateralSolverFilter_3(long j, long j2, long j3, long j4, double d, double d2, double d3);

    private static native void fastBilateralSolverFilter_4(long j, long j2, long j3, long j4, double d, double d2);

    private static native void fastBilateralSolverFilter_5(long j, long j2, long j3, long j4, double d);

    private static native void fastBilateralSolverFilter_6(long j, long j2, long j3, long j4);

    private static native void fastGlobalSmootherFilter_0(long j, long j2, long j3, double d, double d2, double d3, int i);

    private static native void fastGlobalSmootherFilter_1(long j, long j2, long j3, double d, double d2, double d3);

    private static native void fastGlobalSmootherFilter_2(long j, long j2, long j3, double d, double d2);

    private static native void fourierDescriptor_0(long j, long j2, int i, int i2);

    private static native void fourierDescriptor_1(long j, long j2, int i);

    private static native void fourierDescriptor_2(long j, long j2);

    private static native void guidedFilter_0(long j, long j2, long j3, int i, double d, int i2);

    private static native void guidedFilter_1(long j, long j2, long j3, int i, double d);

    private static native void jointBilateralFilter_0(long j, long j2, long j3, int i, double d, double d2, int i2);

    private static native void jointBilateralFilter_1(long j, long j2, long j3, int i, double d, double d2);

    private static native void l0Smooth_0(long j, long j2, double d, double d2);

    private static native void l0Smooth_1(long j, long j2, double d);

    private static native void l0Smooth_2(long j, long j2);

    private static native void niBlackThreshold_0(long j, long j2, double d, int i, int i2, double d2, int i3);

    private static native void niBlackThreshold_1(long j, long j2, double d, int i, int i2, double d2);

    private static native void qconj_0(long j, long j2);

    private static native void qdft_0(long j, long j2, int i, boolean z);

    private static native void qmultiply_0(long j, long j2, long j3);

    private static native void qunitary_0(long j, long j2);

    private static native void rollingGuidanceFilter_0(long j, long j2, int i, double d, double d2, int i2, int i3);

    private static native void rollingGuidanceFilter_1(long j, long j2, int i, double d, double d2, int i2);

    private static native void rollingGuidanceFilter_2(long j, long j2, int i, double d, double d2);

    private static native void rollingGuidanceFilter_3(long j, long j2, int i, double d);

    private static native void rollingGuidanceFilter_4(long j, long j2, int i);

    private static native void rollingGuidanceFilter_5(long j, long j2);

    private static native void thinning_0(long j, long j2, int i);

    private static native void thinning_1(long j, long j2);

    private static native void transformFD_0(long j, long j2, long j3, boolean z);

    private static native void transformFD_1(long j, long j2, long j3);

    private static native void weightedMedianFilter_0(long j, long j2, long j3, int i, double d, int i2, long j4);

    private static native void weightedMedianFilter_1(long j, long j2, long j3, int i, double d, int i2);

    private static native void weightedMedianFilter_2(long j, long j2, long j3, int i, double d);

    private static native void weightedMedianFilter_3(long j, long j2, long j3, int i);

    public static AdaptiveManifoldFilter createAMFilter(double sigma_s, double sigma_r, boolean adjust_outliers) {
        return AdaptiveManifoldFilter.__fromPtr__(createAMFilter_0(sigma_s, sigma_r, adjust_outliers));
    }

    public static AdaptiveManifoldFilter createAMFilter(double sigma_s, double sigma_r) {
        return AdaptiveManifoldFilter.__fromPtr__(createAMFilter_1(sigma_s, sigma_r));
    }

    public static ContourFitting createContourFitting(int ctr, int fd) {
        return ContourFitting.__fromPtr__(createContourFitting_0(ctr, fd));
    }

    public static ContourFitting createContourFitting(int ctr) {
        return ContourFitting.__fromPtr__(createContourFitting_1(ctr));
    }

    public static ContourFitting createContourFitting() {
        return ContourFitting.__fromPtr__(createContourFitting_2());
    }

    public static DTFilter createDTFilter(Mat guide, double sigmaSpatial, double sigmaColor, int mode, int numIters) {
        return DTFilter.__fromPtr__(createDTFilter_0(guide.nativeObj, sigmaSpatial, sigmaColor, mode, numIters));
    }

    public static DTFilter createDTFilter(Mat guide, double sigmaSpatial, double sigmaColor, int mode) {
        return DTFilter.__fromPtr__(createDTFilter_1(guide.nativeObj, sigmaSpatial, sigmaColor, mode));
    }

    public static DTFilter createDTFilter(Mat guide, double sigmaSpatial, double sigmaColor) {
        return DTFilter.__fromPtr__(createDTFilter_2(guide.nativeObj, sigmaSpatial, sigmaColor));
    }

    public static DisparityWLSFilter createDisparityWLSFilter(StereoMatcher matcher_left) {
        return DisparityWLSFilter.__fromPtr__(createDisparityWLSFilter_0(matcher_left.getNativeObjAddr()));
    }

    public static DisparityWLSFilter createDisparityWLSFilterGeneric(boolean use_confidence) {
        return DisparityWLSFilter.__fromPtr__(createDisparityWLSFilterGeneric_0(use_confidence));
    }

    public static EdgeAwareInterpolator createEdgeAwareInterpolator() {
        return EdgeAwareInterpolator.__fromPtr__(createEdgeAwareInterpolator_0());
    }

    public static EdgeBoxes createEdgeBoxes(float alpha, float beta, float eta, float minScore, int maxBoxes, float edgeMinMag, float edgeMergeThr, float clusterMinMag, float maxAspectRatio, float minBoxArea, float gamma, float kappa) {
        return EdgeBoxes.__fromPtr__(createEdgeBoxes_0(alpha, beta, eta, minScore, maxBoxes, edgeMinMag, edgeMergeThr, clusterMinMag, maxAspectRatio, minBoxArea, gamma, kappa));
    }

    public static EdgeBoxes createEdgeBoxes(float alpha, float beta, float eta, float minScore, int maxBoxes, float edgeMinMag, float edgeMergeThr, float clusterMinMag, float maxAspectRatio, float minBoxArea, float gamma) {
        return EdgeBoxes.__fromPtr__(createEdgeBoxes_1(alpha, beta, eta, minScore, maxBoxes, edgeMinMag, edgeMergeThr, clusterMinMag, maxAspectRatio, minBoxArea, gamma));
    }

    public static EdgeBoxes createEdgeBoxes(float alpha, float beta, float eta, float minScore, int maxBoxes, float edgeMinMag, float edgeMergeThr, float clusterMinMag, float maxAspectRatio, float minBoxArea) {
        return EdgeBoxes.__fromPtr__(createEdgeBoxes_2(alpha, beta, eta, minScore, maxBoxes, edgeMinMag, edgeMergeThr, clusterMinMag, maxAspectRatio, minBoxArea));
    }

    public static EdgeBoxes createEdgeBoxes(float alpha, float beta, float eta, float minScore, int maxBoxes, float edgeMinMag, float edgeMergeThr, float clusterMinMag, float maxAspectRatio) {
        return EdgeBoxes.__fromPtr__(createEdgeBoxes_3(alpha, beta, eta, minScore, maxBoxes, edgeMinMag, edgeMergeThr, clusterMinMag, maxAspectRatio));
    }

    public static EdgeBoxes createEdgeBoxes(float alpha, float beta, float eta, float minScore, int maxBoxes, float edgeMinMag, float edgeMergeThr, float clusterMinMag) {
        return EdgeBoxes.__fromPtr__(createEdgeBoxes_4(alpha, beta, eta, minScore, maxBoxes, edgeMinMag, edgeMergeThr, clusterMinMag));
    }

    public static EdgeBoxes createEdgeBoxes(float alpha, float beta, float eta, float minScore, int maxBoxes, float edgeMinMag, float edgeMergeThr) {
        return EdgeBoxes.__fromPtr__(createEdgeBoxes_5(alpha, beta, eta, minScore, maxBoxes, edgeMinMag, edgeMergeThr));
    }

    public static EdgeBoxes createEdgeBoxes(float alpha, float beta, float eta, float minScore, int maxBoxes, float edgeMinMag) {
        return EdgeBoxes.__fromPtr__(createEdgeBoxes_6(alpha, beta, eta, minScore, maxBoxes, edgeMinMag));
    }

    public static EdgeBoxes createEdgeBoxes(float alpha, float beta, float eta, float minScore, int maxBoxes) {
        return EdgeBoxes.__fromPtr__(createEdgeBoxes_7(alpha, beta, eta, minScore, maxBoxes));
    }

    public static EdgeBoxes createEdgeBoxes(float alpha, float beta, float eta, float minScore) {
        return EdgeBoxes.__fromPtr__(createEdgeBoxes_8(alpha, beta, eta, minScore));
    }

    public static EdgeBoxes createEdgeBoxes(float alpha, float beta, float eta) {
        return EdgeBoxes.__fromPtr__(createEdgeBoxes_9(alpha, beta, eta));
    }

    public static EdgeBoxes createEdgeBoxes(float alpha, float beta) {
        return EdgeBoxes.__fromPtr__(createEdgeBoxes_10(alpha, beta));
    }

    public static EdgeBoxes createEdgeBoxes(float alpha) {
        return EdgeBoxes.__fromPtr__(createEdgeBoxes_11(alpha));
    }

    public static EdgeBoxes createEdgeBoxes() {
        return EdgeBoxes.__fromPtr__(createEdgeBoxes_12());
    }

    public static FastBilateralSolverFilter createFastBilateralSolverFilter(Mat guide, double sigma_spatial, double sigma_luma, double sigma_chroma, double lambda, int num_iter, double max_tol) {
        return FastBilateralSolverFilter.__fromPtr__(createFastBilateralSolverFilter_0(guide.nativeObj, sigma_spatial, sigma_luma, sigma_chroma, lambda, num_iter, max_tol));
    }

    public static FastBilateralSolverFilter createFastBilateralSolverFilter(Mat guide, double sigma_spatial, double sigma_luma, double sigma_chroma, double lambda, int num_iter) {
        return FastBilateralSolverFilter.__fromPtr__(createFastBilateralSolverFilter_1(guide.nativeObj, sigma_spatial, sigma_luma, sigma_chroma, lambda, num_iter));
    }

    public static FastBilateralSolverFilter createFastBilateralSolverFilter(Mat guide, double sigma_spatial, double sigma_luma, double sigma_chroma, double lambda) {
        return FastBilateralSolverFilter.__fromPtr__(createFastBilateralSolverFilter_2(guide.nativeObj, sigma_spatial, sigma_luma, sigma_chroma, lambda));
    }

    public static FastBilateralSolverFilter createFastBilateralSolverFilter(Mat guide, double sigma_spatial, double sigma_luma, double sigma_chroma) {
        return FastBilateralSolverFilter.__fromPtr__(createFastBilateralSolverFilter_3(guide.nativeObj, sigma_spatial, sigma_luma, sigma_chroma));
    }

    public static FastGlobalSmootherFilter createFastGlobalSmootherFilter(Mat guide, double lambda, double sigma_color, double lambda_attenuation, int num_iter) {
        return FastGlobalSmootherFilter.__fromPtr__(createFastGlobalSmootherFilter_0(guide.nativeObj, lambda, sigma_color, lambda_attenuation, num_iter));
    }

    public static FastGlobalSmootherFilter createFastGlobalSmootherFilter(Mat guide, double lambda, double sigma_color, double lambda_attenuation) {
        return FastGlobalSmootherFilter.__fromPtr__(createFastGlobalSmootherFilter_1(guide.nativeObj, lambda, sigma_color, lambda_attenuation));
    }

    public static FastGlobalSmootherFilter createFastGlobalSmootherFilter(Mat guide, double lambda, double sigma_color) {
        return FastGlobalSmootherFilter.__fromPtr__(createFastGlobalSmootherFilter_2(guide.nativeObj, lambda, sigma_color));
    }

    public static FastLineDetector createFastLineDetector(int _length_threshold, float _distance_threshold, double _canny_th1, double _canny_th2, int _canny_aperture_size, boolean _do_merge) {
        return FastLineDetector.__fromPtr__(createFastLineDetector_0(_length_threshold, _distance_threshold, _canny_th1, _canny_th2, _canny_aperture_size, _do_merge));
    }

    public static FastLineDetector createFastLineDetector(int _length_threshold, float _distance_threshold, double _canny_th1, double _canny_th2, int _canny_aperture_size) {
        return FastLineDetector.__fromPtr__(createFastLineDetector_1(_length_threshold, _distance_threshold, _canny_th1, _canny_th2, _canny_aperture_size));
    }

    public static FastLineDetector createFastLineDetector(int _length_threshold, float _distance_threshold, double _canny_th1, double _canny_th2) {
        return FastLineDetector.__fromPtr__(createFastLineDetector_2(_length_threshold, _distance_threshold, _canny_th1, _canny_th2));
    }

    public static FastLineDetector createFastLineDetector(int _length_threshold, float _distance_threshold, double _canny_th1) {
        return FastLineDetector.__fromPtr__(createFastLineDetector_3(_length_threshold, _distance_threshold, _canny_th1));
    }

    public static FastLineDetector createFastLineDetector(int _length_threshold, float _distance_threshold) {
        return FastLineDetector.__fromPtr__(createFastLineDetector_4(_length_threshold, _distance_threshold));
    }

    public static FastLineDetector createFastLineDetector(int _length_threshold) {
        return FastLineDetector.__fromPtr__(createFastLineDetector_5(_length_threshold));
    }

    public static FastLineDetector createFastLineDetector() {
        return FastLineDetector.__fromPtr__(createFastLineDetector_6());
    }

    public static GraphSegmentation createGraphSegmentation(double sigma, float k, int min_size) {
        return GraphSegmentation.__fromPtr__(createGraphSegmentation_0(sigma, k, min_size));
    }

    public static GraphSegmentation createGraphSegmentation(double sigma, float k) {
        return GraphSegmentation.__fromPtr__(createGraphSegmentation_1(sigma, k));
    }

    public static GraphSegmentation createGraphSegmentation(double sigma) {
        return GraphSegmentation.__fromPtr__(createGraphSegmentation_2(sigma));
    }

    public static GraphSegmentation createGraphSegmentation() {
        return GraphSegmentation.__fromPtr__(createGraphSegmentation_3());
    }

    public static GuidedFilter createGuidedFilter(Mat guide, int radius, double eps) {
        return GuidedFilter.__fromPtr__(createGuidedFilter_0(guide.nativeObj, radius, eps));
    }

    public static RFFeatureGetter createRFFeatureGetter() {
        return RFFeatureGetter.__fromPtr__(createRFFeatureGetter_0());
    }

    public static SelectiveSearchSegmentation createSelectiveSearchSegmentation() {
        return SelectiveSearchSegmentation.__fromPtr__(createSelectiveSearchSegmentation_0());
    }

    public static SelectiveSearchSegmentationStrategyColor createSelectiveSearchSegmentationStrategyColor() {
        return SelectiveSearchSegmentationStrategyColor.__fromPtr__(createSelectiveSearchSegmentationStrategyColor_0());
    }

    public static SelectiveSearchSegmentationStrategyFill createSelectiveSearchSegmentationStrategyFill() {
        return SelectiveSearchSegmentationStrategyFill.__fromPtr__(createSelectiveSearchSegmentationStrategyFill_0());
    }

    public static SelectiveSearchSegmentationStrategyMultiple createSelectiveSearchSegmentationStrategyMultiple(SelectiveSearchSegmentationStrategy s1, SelectiveSearchSegmentationStrategy s2, SelectiveSearchSegmentationStrategy s3, SelectiveSearchSegmentationStrategy s4) {
        return SelectiveSearchSegmentationStrategyMultiple.__fromPtr__(createSelectiveSearchSegmentationStrategyMultiple_0(s1.getNativeObjAddr(), s2.getNativeObjAddr(), s3.getNativeObjAddr(), s4.getNativeObjAddr()));
    }

    public static SelectiveSearchSegmentationStrategyMultiple createSelectiveSearchSegmentationStrategyMultiple(SelectiveSearchSegmentationStrategy s1, SelectiveSearchSegmentationStrategy s2, SelectiveSearchSegmentationStrategy s3) {
        return SelectiveSearchSegmentationStrategyMultiple.__fromPtr__(createSelectiveSearchSegmentationStrategyMultiple_1(s1.getNativeObjAddr(), s2.getNativeObjAddr(), s3.getNativeObjAddr()));
    }

    public static SelectiveSearchSegmentationStrategyMultiple createSelectiveSearchSegmentationStrategyMultiple(SelectiveSearchSegmentationStrategy s1, SelectiveSearchSegmentationStrategy s2) {
        return SelectiveSearchSegmentationStrategyMultiple.__fromPtr__(createSelectiveSearchSegmentationStrategyMultiple_2(s1.getNativeObjAddr(), s2.getNativeObjAddr()));
    }

    public static SelectiveSearchSegmentationStrategyMultiple createSelectiveSearchSegmentationStrategyMultiple(SelectiveSearchSegmentationStrategy s1) {
        return SelectiveSearchSegmentationStrategyMultiple.__fromPtr__(createSelectiveSearchSegmentationStrategyMultiple_3(s1.getNativeObjAddr()));
    }

    public static SelectiveSearchSegmentationStrategyMultiple createSelectiveSearchSegmentationStrategyMultiple() {
        return SelectiveSearchSegmentationStrategyMultiple.__fromPtr__(createSelectiveSearchSegmentationStrategyMultiple_4());
    }

    public static SelectiveSearchSegmentationStrategySize createSelectiveSearchSegmentationStrategySize() {
        return SelectiveSearchSegmentationStrategySize.__fromPtr__(createSelectiveSearchSegmentationStrategySize_0());
    }

    public static SelectiveSearchSegmentationStrategyTexture createSelectiveSearchSegmentationStrategyTexture() {
        return SelectiveSearchSegmentationStrategyTexture.__fromPtr__(createSelectiveSearchSegmentationStrategyTexture_0());
    }

    public static StereoMatcher createRightMatcher(StereoMatcher matcher_left) {
        return StereoMatcher.__fromPtr__(createRightMatcher_0(matcher_left.getNativeObjAddr()));
    }

    public static StructuredEdgeDetection createStructuredEdgeDetection(String model, RFFeatureGetter howToGetFeatures) {
        return StructuredEdgeDetection.__fromPtr__(createStructuredEdgeDetection_0(model, howToGetFeatures.getNativeObjAddr()));
    }

    public static StructuredEdgeDetection createStructuredEdgeDetection(String model) {
        return StructuredEdgeDetection.__fromPtr__(createStructuredEdgeDetection_1(model));
    }

    public static SuperpixelLSC createSuperpixelLSC(Mat image, int region_size, float ratio) {
        return SuperpixelLSC.__fromPtr__(createSuperpixelLSC_0(image.nativeObj, region_size, ratio));
    }

    public static SuperpixelLSC createSuperpixelLSC(Mat image, int region_size) {
        return SuperpixelLSC.__fromPtr__(createSuperpixelLSC_1(image.nativeObj, region_size));
    }

    public static SuperpixelLSC createSuperpixelLSC(Mat image) {
        return SuperpixelLSC.__fromPtr__(createSuperpixelLSC_2(image.nativeObj));
    }

    public static SuperpixelSEEDS createSuperpixelSEEDS(int image_width, int image_height, int image_channels, int num_superpixels, int num_levels, int prior, int histogram_bins, boolean double_step) {
        return SuperpixelSEEDS.__fromPtr__(createSuperpixelSEEDS_0(image_width, image_height, image_channels, num_superpixels, num_levels, prior, histogram_bins, double_step));
    }

    public static SuperpixelSEEDS createSuperpixelSEEDS(int image_width, int image_height, int image_channels, int num_superpixels, int num_levels, int prior, int histogram_bins) {
        return SuperpixelSEEDS.__fromPtr__(createSuperpixelSEEDS_1(image_width, image_height, image_channels, num_superpixels, num_levels, prior, histogram_bins));
    }

    public static SuperpixelSEEDS createSuperpixelSEEDS(int image_width, int image_height, int image_channels, int num_superpixels, int num_levels, int prior) {
        return SuperpixelSEEDS.__fromPtr__(createSuperpixelSEEDS_2(image_width, image_height, image_channels, num_superpixels, num_levels, prior));
    }

    public static SuperpixelSEEDS createSuperpixelSEEDS(int image_width, int image_height, int image_channels, int num_superpixels, int num_levels) {
        return SuperpixelSEEDS.__fromPtr__(createSuperpixelSEEDS_3(image_width, image_height, image_channels, num_superpixels, num_levels));
    }

    public static SuperpixelSLIC createSuperpixelSLIC(Mat image, int algorithm, int region_size, float ruler) {
        return SuperpixelSLIC.__fromPtr__(createSuperpixelSLIC_0(image.nativeObj, algorithm, region_size, ruler));
    }

    public static SuperpixelSLIC createSuperpixelSLIC(Mat image, int algorithm, int region_size) {
        return SuperpixelSLIC.__fromPtr__(createSuperpixelSLIC_1(image.nativeObj, algorithm, region_size));
    }

    public static SuperpixelSLIC createSuperpixelSLIC(Mat image, int algorithm) {
        return SuperpixelSLIC.__fromPtr__(createSuperpixelSLIC_2(image.nativeObj, algorithm));
    }

    public static SuperpixelSLIC createSuperpixelSLIC(Mat image) {
        return SuperpixelSLIC.__fromPtr__(createSuperpixelSLIC_3(image.nativeObj));
    }

    public static void FastHoughTransform(Mat src, Mat dst, int dstMatDepth, int angleRange, int op, int makeSkew) {
        FastHoughTransform_0(src.nativeObj, dst.nativeObj, dstMatDepth, angleRange, op, makeSkew);
    }

    public static void FastHoughTransform(Mat src, Mat dst, int dstMatDepth, int angleRange, int op) {
        FastHoughTransform_1(src.nativeObj, dst.nativeObj, dstMatDepth, angleRange, op);
    }

    public static void FastHoughTransform(Mat src, Mat dst, int dstMatDepth, int angleRange) {
        FastHoughTransform_2(src.nativeObj, dst.nativeObj, dstMatDepth, angleRange);
    }

    public static void FastHoughTransform(Mat src, Mat dst, int dstMatDepth) {
        FastHoughTransform_3(src.nativeObj, dst.nativeObj, dstMatDepth);
    }

    public static void GradientDericheX(Mat op, Mat dst, double alpha, double omega) {
        GradientDericheX_0(op.nativeObj, dst.nativeObj, alpha, omega);
    }

    public static void GradientDericheY(Mat op, Mat dst, double alpha, double omega) {
        GradientDericheY_0(op.nativeObj, dst.nativeObj, alpha, omega);
    }

    public static void PeiLinNormalization(Mat I, Mat T) {
        PeiLinNormalization_0(I.nativeObj, T.nativeObj);
    }

    public static void amFilter(Mat joint, Mat src, Mat dst, double sigma_s, double sigma_r, boolean adjust_outliers) {
        amFilter_0(joint.nativeObj, src.nativeObj, dst.nativeObj, sigma_s, sigma_r, adjust_outliers);
    }

    public static void amFilter(Mat joint, Mat src, Mat dst, double sigma_s, double sigma_r) {
        amFilter_1(joint.nativeObj, src.nativeObj, dst.nativeObj, sigma_s, sigma_r);
    }

    public static void anisotropicDiffusion(Mat src, Mat dst, float alpha, float K, int niters) {
        anisotropicDiffusion_0(src.nativeObj, dst.nativeObj, alpha, K, niters);
    }

    public static void bilateralTextureFilter(Mat src, Mat dst, int fr, int numIter, double sigmaAlpha, double sigmaAvg) {
        bilateralTextureFilter_0(src.nativeObj, dst.nativeObj, fr, numIter, sigmaAlpha, sigmaAvg);
    }

    public static void bilateralTextureFilter(Mat src, Mat dst, int fr, int numIter, double sigmaAlpha) {
        bilateralTextureFilter_1(src.nativeObj, dst.nativeObj, fr, numIter, sigmaAlpha);
    }

    public static void bilateralTextureFilter(Mat src, Mat dst, int fr, int numIter) {
        bilateralTextureFilter_2(src.nativeObj, dst.nativeObj, fr, numIter);
    }

    public static void bilateralTextureFilter(Mat src, Mat dst, int fr) {
        bilateralTextureFilter_3(src.nativeObj, dst.nativeObj, fr);
    }

    public static void bilateralTextureFilter(Mat src, Mat dst) {
        bilateralTextureFilter_4(src.nativeObj, dst.nativeObj);
    }

    public static void colorMatchTemplate(Mat img, Mat templ, Mat result) {
        colorMatchTemplate_0(img.nativeObj, templ.nativeObj, result.nativeObj);
    }

    public static void contourSampling(Mat src, Mat out, int nbElt) {
        contourSampling_0(src.nativeObj, out.nativeObj, nbElt);
    }

    public static void covarianceEstimation(Mat src, Mat dst, int windowRows, int windowCols) {
        covarianceEstimation_0(src.nativeObj, dst.nativeObj, windowRows, windowCols);
    }

    public static void createQuaternionImage(Mat img, Mat qimg) {
        createQuaternionImage_0(img.nativeObj, qimg.nativeObj);
    }

    public static void dtFilter(Mat guide, Mat src, Mat dst, double sigmaSpatial, double sigmaColor, int mode, int numIters) {
        dtFilter_0(guide.nativeObj, src.nativeObj, dst.nativeObj, sigmaSpatial, sigmaColor, mode, numIters);
    }

    public static void dtFilter(Mat guide, Mat src, Mat dst, double sigmaSpatial, double sigmaColor, int mode) {
        dtFilter_1(guide.nativeObj, src.nativeObj, dst.nativeObj, sigmaSpatial, sigmaColor, mode);
    }

    public static void dtFilter(Mat guide, Mat src, Mat dst, double sigmaSpatial, double sigmaColor) {
        dtFilter_2(guide.nativeObj, src.nativeObj, dst.nativeObj, sigmaSpatial, sigmaColor);
    }

    public static void edgePreservingFilter(Mat src, Mat dst, int d, double threshold) {
        edgePreservingFilter_0(src.nativeObj, dst.nativeObj, d, threshold);
    }

    public static void fastBilateralSolverFilter(Mat guide, Mat src, Mat confidence, Mat dst, double sigma_spatial, double sigma_luma, double sigma_chroma, double lambda, int num_iter, double max_tol) {
        long j = guide.nativeObj;
        long j2 = j;
        fastBilateralSolverFilter_0(j2, src.nativeObj, confidence.nativeObj, dst.nativeObj, sigma_spatial, sigma_luma, sigma_chroma, lambda, num_iter, max_tol);
    }

    public static void fastBilateralSolverFilter(Mat guide, Mat src, Mat confidence, Mat dst, double sigma_spatial, double sigma_luma, double sigma_chroma, double lambda, int num_iter) {
        long j = guide.nativeObj;
        long j2 = j;
        fastBilateralSolverFilter_1(j2, src.nativeObj, confidence.nativeObj, dst.nativeObj, sigma_spatial, sigma_luma, sigma_chroma, lambda, num_iter);
    }

    public static void fastBilateralSolverFilter(Mat guide, Mat src, Mat confidence, Mat dst, double sigma_spatial, double sigma_luma, double sigma_chroma, double lambda) {
        fastBilateralSolverFilter_2(guide.nativeObj, src.nativeObj, confidence.nativeObj, dst.nativeObj, sigma_spatial, sigma_luma, sigma_chroma, lambda);
    }

    public static void fastBilateralSolverFilter(Mat guide, Mat src, Mat confidence, Mat dst, double sigma_spatial, double sigma_luma, double sigma_chroma) {
        fastBilateralSolverFilter_3(guide.nativeObj, src.nativeObj, confidence.nativeObj, dst.nativeObj, sigma_spatial, sigma_luma, sigma_chroma);
    }

    public static void fastBilateralSolverFilter(Mat guide, Mat src, Mat confidence, Mat dst, double sigma_spatial, double sigma_luma) {
        fastBilateralSolverFilter_4(guide.nativeObj, src.nativeObj, confidence.nativeObj, dst.nativeObj, sigma_spatial, sigma_luma);
    }

    public static void fastBilateralSolverFilter(Mat guide, Mat src, Mat confidence, Mat dst, double sigma_spatial) {
        fastBilateralSolverFilter_5(guide.nativeObj, src.nativeObj, confidence.nativeObj, dst.nativeObj, sigma_spatial);
    }

    public static void fastBilateralSolverFilter(Mat guide, Mat src, Mat confidence, Mat dst) {
        fastBilateralSolverFilter_6(guide.nativeObj, src.nativeObj, confidence.nativeObj, dst.nativeObj);
    }

    public static void fastGlobalSmootherFilter(Mat guide, Mat src, Mat dst, double lambda, double sigma_color, double lambda_attenuation, int num_iter) {
        fastGlobalSmootherFilter_0(guide.nativeObj, src.nativeObj, dst.nativeObj, lambda, sigma_color, lambda_attenuation, num_iter);
    }

    public static void fastGlobalSmootherFilter(Mat guide, Mat src, Mat dst, double lambda, double sigma_color, double lambda_attenuation) {
        fastGlobalSmootherFilter_1(guide.nativeObj, src.nativeObj, dst.nativeObj, lambda, sigma_color, lambda_attenuation);
    }

    public static void fastGlobalSmootherFilter(Mat guide, Mat src, Mat dst, double lambda, double sigma_color) {
        fastGlobalSmootherFilter_2(guide.nativeObj, src.nativeObj, dst.nativeObj, lambda, sigma_color);
    }

    public static void fourierDescriptor(Mat src, Mat dst, int nbElt, int nbFD) {
        fourierDescriptor_0(src.nativeObj, dst.nativeObj, nbElt, nbFD);
    }

    public static void fourierDescriptor(Mat src, Mat dst, int nbElt) {
        fourierDescriptor_1(src.nativeObj, dst.nativeObj, nbElt);
    }

    public static void fourierDescriptor(Mat src, Mat dst) {
        fourierDescriptor_2(src.nativeObj, dst.nativeObj);
    }

    public static void guidedFilter(Mat guide, Mat src, Mat dst, int radius, double eps, int dDepth) {
        guidedFilter_0(guide.nativeObj, src.nativeObj, dst.nativeObj, radius, eps, dDepth);
    }

    public static void guidedFilter(Mat guide, Mat src, Mat dst, int radius, double eps) {
        guidedFilter_1(guide.nativeObj, src.nativeObj, dst.nativeObj, radius, eps);
    }

    public static void jointBilateralFilter(Mat joint, Mat src, Mat dst, int d, double sigmaColor, double sigmaSpace, int borderType) {
        jointBilateralFilter_0(joint.nativeObj, src.nativeObj, dst.nativeObj, d, sigmaColor, sigmaSpace, borderType);
    }

    public static void jointBilateralFilter(Mat joint, Mat src, Mat dst, int d, double sigmaColor, double sigmaSpace) {
        jointBilateralFilter_1(joint.nativeObj, src.nativeObj, dst.nativeObj, d, sigmaColor, sigmaSpace);
    }

    public static void l0Smooth(Mat src, Mat dst, double lambda, double kappa) {
        l0Smooth_0(src.nativeObj, dst.nativeObj, lambda, kappa);
    }

    public static void l0Smooth(Mat src, Mat dst, double lambda) {
        l0Smooth_1(src.nativeObj, dst.nativeObj, lambda);
    }

    public static void l0Smooth(Mat src, Mat dst) {
        l0Smooth_2(src.nativeObj, dst.nativeObj);
    }

    public static void niBlackThreshold(Mat _src, Mat _dst, double maxValue, int type, int blockSize, double k, int binarizationMethod) {
        niBlackThreshold_0(_src.nativeObj, _dst.nativeObj, maxValue, type, blockSize, k, binarizationMethod);
    }

    public static void niBlackThreshold(Mat _src, Mat _dst, double maxValue, int type, int blockSize, double k) {
        niBlackThreshold_1(_src.nativeObj, _dst.nativeObj, maxValue, type, blockSize, k);
    }

    public static void qconj(Mat qimg, Mat qcimg) {
        qconj_0(qimg.nativeObj, qcimg.nativeObj);
    }

    public static void qdft(Mat img, Mat qimg, int flags, boolean sideLeft) {
        qdft_0(img.nativeObj, qimg.nativeObj, flags, sideLeft);
    }

    public static void qmultiply(Mat src1, Mat src2, Mat dst) {
        qmultiply_0(src1.nativeObj, src2.nativeObj, dst.nativeObj);
    }

    public static void qunitary(Mat qimg, Mat qnimg) {
        qunitary_0(qimg.nativeObj, qnimg.nativeObj);
    }

    public static void rollingGuidanceFilter(Mat src, Mat dst, int d, double sigmaColor, double sigmaSpace, int numOfIter, int borderType) {
        rollingGuidanceFilter_0(src.nativeObj, dst.nativeObj, d, sigmaColor, sigmaSpace, numOfIter, borderType);
    }

    public static void rollingGuidanceFilter(Mat src, Mat dst, int d, double sigmaColor, double sigmaSpace, int numOfIter) {
        rollingGuidanceFilter_1(src.nativeObj, dst.nativeObj, d, sigmaColor, sigmaSpace, numOfIter);
    }

    public static void rollingGuidanceFilter(Mat src, Mat dst, int d, double sigmaColor, double sigmaSpace) {
        rollingGuidanceFilter_2(src.nativeObj, dst.nativeObj, d, sigmaColor, sigmaSpace);
    }

    public static void rollingGuidanceFilter(Mat src, Mat dst, int d, double sigmaColor) {
        rollingGuidanceFilter_3(src.nativeObj, dst.nativeObj, d, sigmaColor);
    }

    public static void rollingGuidanceFilter(Mat src, Mat dst, int d) {
        rollingGuidanceFilter_4(src.nativeObj, dst.nativeObj, d);
    }

    public static void rollingGuidanceFilter(Mat src, Mat dst) {
        rollingGuidanceFilter_5(src.nativeObj, dst.nativeObj);
    }

    public static void thinning(Mat src, Mat dst, int thinningType) {
        thinning_0(src.nativeObj, dst.nativeObj, thinningType);
    }

    public static void thinning(Mat src, Mat dst) {
        thinning_1(src.nativeObj, dst.nativeObj);
    }

    public static void transformFD(Mat src, Mat t, Mat dst, boolean fdContour) {
        transformFD_0(src.nativeObj, t.nativeObj, dst.nativeObj, fdContour);
    }

    public static void transformFD(Mat src, Mat t, Mat dst) {
        transformFD_1(src.nativeObj, t.nativeObj, dst.nativeObj);
    }

    public static void weightedMedianFilter(Mat joint, Mat src, Mat dst, int r, double sigma, int weightType, Mat mask) {
        weightedMedianFilter_0(joint.nativeObj, src.nativeObj, dst.nativeObj, r, sigma, weightType, mask.nativeObj);
    }

    public static void weightedMedianFilter(Mat joint, Mat src, Mat dst, int r, double sigma, int weightType) {
        weightedMedianFilter_1(joint.nativeObj, src.nativeObj, dst.nativeObj, r, sigma, weightType);
    }

    public static void weightedMedianFilter(Mat joint, Mat src, Mat dst, int r, double sigma) {
        weightedMedianFilter_2(joint.nativeObj, src.nativeObj, dst.nativeObj, r, sigma);
    }

    public static void weightedMedianFilter(Mat joint, Mat src, Mat dst, int r) {
        weightedMedianFilter_3(joint.nativeObj, src.nativeObj, dst.nativeObj, r);
    }
}
