package org.bytedeco.javacpp;

import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Const;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.annotation.Opaque;
import org.bytedeco.javacpp.opencv_calib3d;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.presets.opencv_core;

public class opencv_ximgproc extends org.bytedeco.javacpp.presets.opencv_ximgproc {
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
    public static final int RO_IGNORE_BORDERS = 1;
    public static final int RO_STRICT = 0;
    public static final int SLIC = 100;
    public static final int SLICO = 101;
    public static final int THINNING_GUOHALL = 1;
    public static final int THINNING_ZHANGSUEN = 0;

    @Namespace("cv::ximgproc")
    public static native void FastHoughTransform(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i);

    @Namespace("cv::ximgproc")
    public static native void FastHoughTransform(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, int i2, int i3, int i4);

    @Namespace("cv::ximgproc")
    public static native void FastHoughTransform(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i);

    @Namespace("cv::ximgproc")
    public static native void FastHoughTransform(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, int i2, int i3, int i4);

    @Namespace("cv::ximgproc")
    public static native void FastHoughTransform(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i);

    @Namespace("cv::ximgproc")
    public static native void FastHoughTransform(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, int i2, int i3, int i4);

    @Namespace("cv::ximgproc")
    @ByVal
    public static native opencv_core.Scalar4i HoughPoint2Line(@ByRef @Const opencv_core.Point point, @ByVal opencv_core.GpuMat gpuMat);

    @Namespace("cv::ximgproc")
    @ByVal
    public static native opencv_core.Scalar4i HoughPoint2Line(@ByRef @Const opencv_core.Point point, @ByVal opencv_core.GpuMat gpuMat, int i, int i2, int i3);

    @Namespace("cv::ximgproc")
    @ByVal
    public static native opencv_core.Scalar4i HoughPoint2Line(@ByRef @Const opencv_core.Point point, @ByVal opencv_core.Mat mat);

    @Namespace("cv::ximgproc")
    @ByVal
    public static native opencv_core.Scalar4i HoughPoint2Line(@ByRef @Const opencv_core.Point point, @ByVal opencv_core.Mat mat, int i, int i2, int i3);

    @Namespace("cv::ximgproc")
    @ByVal
    public static native opencv_core.Scalar4i HoughPoint2Line(@ByRef @Const opencv_core.Point point, @ByVal opencv_core.UMat uMat);

    @Namespace("cv::ximgproc")
    @ByVal
    public static native opencv_core.Scalar4i HoughPoint2Line(@ByRef @Const opencv_core.Point point, @ByVal opencv_core.UMat uMat, int i, int i2, int i3);

    @Namespace("cv::ximgproc")
    public static native void amFilter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, double d, double d2);

    @Namespace("cv::ximgproc")
    public static native void amFilter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, double d, double d2, @Cast({"bool"}) boolean z);

    @Namespace("cv::ximgproc")
    public static native void amFilter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, double d, double d2);

    @Namespace("cv::ximgproc")
    public static native void amFilter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, double d, double d2, @Cast({"bool"}) boolean z);

    @Namespace("cv::ximgproc")
    public static native void amFilter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, double d, double d2);

    @Namespace("cv::ximgproc")
    public static native void amFilter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, double d, double d2, @Cast({"bool"}) boolean z);

    @Namespace("cv::ximgproc")
    public static native void anisotropicDiffusion(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, float f, float f2, int i);

    @Namespace("cv::ximgproc")
    public static native void anisotropicDiffusion(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, float f, float f2, int i);

    @Namespace("cv::ximgproc")
    public static native void anisotropicDiffusion(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, float f, float f2, int i);

    @Namespace("cv::ximgproc")
    public static native void bilateralTextureFilter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::ximgproc")
    public static native void bilateralTextureFilter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, int i2, double d, double d2);

    @Namespace("cv::ximgproc")
    public static native void bilateralTextureFilter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::ximgproc")
    public static native void bilateralTextureFilter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, int i2, double d, double d2);

    @Namespace("cv::ximgproc")
    public static native void bilateralTextureFilter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::ximgproc")
    public static native void bilateralTextureFilter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, int i2, double d, double d2);

    @Namespace("cv::ximgproc")
    public static native double computeBadPixelPercent(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.Rect rect);

    @Namespace("cv::ximgproc")
    public static native double computeBadPixelPercent(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.Rect rect, int i);

    @Namespace("cv::ximgproc")
    public static native double computeBadPixelPercent(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Rect rect);

    @Namespace("cv::ximgproc")
    public static native double computeBadPixelPercent(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Rect rect, int i);

    @Namespace("cv::ximgproc")
    public static native double computeBadPixelPercent(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.Rect rect);

    @Namespace("cv::ximgproc")
    public static native double computeBadPixelPercent(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.Rect rect, int i);

    @Namespace("cv::ximgproc")
    public static native double computeMSE(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.Rect rect);

    @Namespace("cv::ximgproc")
    public static native double computeMSE(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Rect rect);

    @Namespace("cv::ximgproc")
    public static native double computeMSE(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.Rect rect);

    @Namespace("cv::ximgproc")
    public static native void covarianceEstimation(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, int i2);

    @Namespace("cv::ximgproc")
    public static native void covarianceEstimation(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, int i2);

    @Namespace("cv::ximgproc")
    public static native void covarianceEstimation(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, int i2);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native AdaptiveManifoldFilter createAMFilter(double d, double d2);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native AdaptiveManifoldFilter createAMFilter(double d, double d2, @Cast({"bool"}) boolean z);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native DTFilter createDTFilter(@ByVal opencv_core.GpuMat gpuMat, double d, double d2);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native DTFilter createDTFilter(@ByVal opencv_core.GpuMat gpuMat, double d, double d2, int i, int i2);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native DTFilter createDTFilter(@ByVal opencv_core.Mat mat, double d, double d2);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native DTFilter createDTFilter(@ByVal opencv_core.Mat mat, double d, double d2, int i, int i2);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native DTFilter createDTFilter(@ByVal opencv_core.UMat uMat, double d, double d2);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native DTFilter createDTFilter(@ByVal opencv_core.UMat uMat, double d, double d2, int i, int i2);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native DisparityWLSFilter createDisparityWLSFilter(@opencv_core.Ptr opencv_calib3d.StereoMatcher stereoMatcher);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native DisparityWLSFilter createDisparityWLSFilterGeneric(@Cast({"bool"}) boolean z);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native EdgeAwareInterpolator createEdgeAwareInterpolator();

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native FastBilateralSolverFilter createFastBilateralSolverFilter(@ByVal opencv_core.GpuMat gpuMat, double d, double d2, double d3);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native FastBilateralSolverFilter createFastBilateralSolverFilter(@ByVal opencv_core.GpuMat gpuMat, double d, double d2, double d3, double d4, int i, double d5);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native FastBilateralSolverFilter createFastBilateralSolverFilter(@ByVal opencv_core.Mat mat, double d, double d2, double d3);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native FastBilateralSolverFilter createFastBilateralSolverFilter(@ByVal opencv_core.Mat mat, double d, double d2, double d3, double d4, int i, double d5);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native FastBilateralSolverFilter createFastBilateralSolverFilter(@ByVal opencv_core.UMat uMat, double d, double d2, double d3);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native FastBilateralSolverFilter createFastBilateralSolverFilter(@ByVal opencv_core.UMat uMat, double d, double d2, double d3, double d4, int i, double d5);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native FastGlobalSmootherFilter createFastGlobalSmootherFilter(@ByVal opencv_core.GpuMat gpuMat, double d, double d2);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native FastGlobalSmootherFilter createFastGlobalSmootherFilter(@ByVal opencv_core.GpuMat gpuMat, double d, double d2, double d3, int i);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native FastGlobalSmootherFilter createFastGlobalSmootherFilter(@ByVal opencv_core.Mat mat, double d, double d2);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native FastGlobalSmootherFilter createFastGlobalSmootherFilter(@ByVal opencv_core.Mat mat, double d, double d2, double d3, int i);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native FastGlobalSmootherFilter createFastGlobalSmootherFilter(@ByVal opencv_core.UMat uMat, double d, double d2);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native FastGlobalSmootherFilter createFastGlobalSmootherFilter(@ByVal opencv_core.UMat uMat, double d, double d2, double d3, int i);

    @Namespace("cv::ximgproc::segmentation")
    @opencv_core.Ptr
    public static native GraphSegmentation createGraphSegmentation();

    @Namespace("cv::ximgproc::segmentation")
    @opencv_core.Ptr
    public static native GraphSegmentation createGraphSegmentation(double d, float f, int i);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native GuidedFilter createGuidedFilter(@ByVal opencv_core.GpuMat gpuMat, int i, double d);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native GuidedFilter createGuidedFilter(@ByVal opencv_core.Mat mat, int i, double d);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native GuidedFilter createGuidedFilter(@ByVal opencv_core.UMat uMat, int i, double d);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native RFFeatureGetter createRFFeatureGetter();

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native opencv_calib3d.StereoMatcher createRightMatcher(@opencv_core.Ptr opencv_calib3d.StereoMatcher stereoMatcher);

    @Namespace("cv::ximgproc::segmentation")
    @opencv_core.Ptr
    public static native SelectiveSearchSegmentation createSelectiveSearchSegmentation();

    @Namespace("cv::ximgproc::segmentation")
    @opencv_core.Ptr
    public static native SelectiveSearchSegmentationStrategyColor createSelectiveSearchSegmentationStrategyColor();

    @Namespace("cv::ximgproc::segmentation")
    @opencv_core.Ptr
    public static native SelectiveSearchSegmentationStrategyFill createSelectiveSearchSegmentationStrategyFill();

    @Namespace("cv::ximgproc::segmentation")
    @opencv_core.Ptr
    public static native SelectiveSearchSegmentationStrategyMultiple createSelectiveSearchSegmentationStrategyMultiple();

    @Namespace("cv::ximgproc::segmentation")
    @opencv_core.Ptr
    public static native SelectiveSearchSegmentationStrategyMultiple createSelectiveSearchSegmentationStrategyMultiple(@opencv_core.Ptr SelectiveSearchSegmentationStrategy selectiveSearchSegmentationStrategy);

    @Namespace("cv::ximgproc::segmentation")
    @opencv_core.Ptr
    public static native SelectiveSearchSegmentationStrategyMultiple createSelectiveSearchSegmentationStrategyMultiple(@opencv_core.Ptr SelectiveSearchSegmentationStrategy selectiveSearchSegmentationStrategy, @opencv_core.Ptr SelectiveSearchSegmentationStrategy selectiveSearchSegmentationStrategy2);

    @Namespace("cv::ximgproc::segmentation")
    @opencv_core.Ptr
    public static native SelectiveSearchSegmentationStrategyMultiple createSelectiveSearchSegmentationStrategyMultiple(@opencv_core.Ptr SelectiveSearchSegmentationStrategy selectiveSearchSegmentationStrategy, @opencv_core.Ptr SelectiveSearchSegmentationStrategy selectiveSearchSegmentationStrategy2, @opencv_core.Ptr SelectiveSearchSegmentationStrategy selectiveSearchSegmentationStrategy3);

    @Namespace("cv::ximgproc::segmentation")
    @opencv_core.Ptr
    public static native SelectiveSearchSegmentationStrategyMultiple createSelectiveSearchSegmentationStrategyMultiple(@opencv_core.Ptr SelectiveSearchSegmentationStrategy selectiveSearchSegmentationStrategy, @opencv_core.Ptr SelectiveSearchSegmentationStrategy selectiveSearchSegmentationStrategy2, @opencv_core.Ptr SelectiveSearchSegmentationStrategy selectiveSearchSegmentationStrategy3, @opencv_core.Ptr SelectiveSearchSegmentationStrategy selectiveSearchSegmentationStrategy4);

    @Namespace("cv::ximgproc::segmentation")
    @opencv_core.Ptr
    public static native SelectiveSearchSegmentationStrategySize createSelectiveSearchSegmentationStrategySize();

    @Namespace("cv::ximgproc::segmentation")
    @opencv_core.Ptr
    public static native SelectiveSearchSegmentationStrategyTexture createSelectiveSearchSegmentationStrategyTexture();

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native StructuredEdgeDetection createStructuredEdgeDetection(@opencv_core.Str String str);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native StructuredEdgeDetection createStructuredEdgeDetection(@opencv_core.Str String str, @opencv_core.Ptr @Const RFFeatureGetter rFFeatureGetter);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native StructuredEdgeDetection createStructuredEdgeDetection(@opencv_core.Str BytePointer bytePointer);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native StructuredEdgeDetection createStructuredEdgeDetection(@opencv_core.Str BytePointer bytePointer, @opencv_core.Ptr @Const RFFeatureGetter rFFeatureGetter);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native SuperpixelLSC createSuperpixelLSC(@ByVal opencv_core.GpuMat gpuMat);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native SuperpixelLSC createSuperpixelLSC(@ByVal opencv_core.GpuMat gpuMat, int i, float f);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native SuperpixelLSC createSuperpixelLSC(@ByVal opencv_core.Mat mat);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native SuperpixelLSC createSuperpixelLSC(@ByVal opencv_core.Mat mat, int i, float f);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native SuperpixelLSC createSuperpixelLSC(@ByVal opencv_core.UMat uMat);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native SuperpixelLSC createSuperpixelLSC(@ByVal opencv_core.UMat uMat, int i, float f);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native SuperpixelSEEDS createSuperpixelSEEDS(int i, int i2, int i3, int i4, int i5);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native SuperpixelSEEDS createSuperpixelSEEDS(int i, int i2, int i3, int i4, int i5, int i6, int i7, @Cast({"bool"}) boolean z);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native SuperpixelSLIC createSuperpixelSLIC(@ByVal opencv_core.GpuMat gpuMat);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native SuperpixelSLIC createSuperpixelSLIC(@ByVal opencv_core.GpuMat gpuMat, int i, int i2, float f);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native SuperpixelSLIC createSuperpixelSLIC(@ByVal opencv_core.Mat mat);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native SuperpixelSLIC createSuperpixelSLIC(@ByVal opencv_core.Mat mat, int i, int i2, float f);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native SuperpixelSLIC createSuperpixelSLIC(@ByVal opencv_core.UMat uMat);

    @Namespace("cv::ximgproc")
    @opencv_core.Ptr
    public static native SuperpixelSLIC createSuperpixelSLIC(@ByVal opencv_core.UMat uMat, int i, int i2, float f);

    @Namespace("cv::ximgproc")
    public static native void dtFilter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, double d, double d2);

    @Namespace("cv::ximgproc")
    public static native void dtFilter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, double d, double d2, int i, int i2);

    @Namespace("cv::ximgproc")
    public static native void dtFilter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, double d, double d2);

    @Namespace("cv::ximgproc")
    public static native void dtFilter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, double d, double d2, int i, int i2);

    @Namespace("cv::ximgproc")
    public static native void dtFilter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, double d, double d2);

    @Namespace("cv::ximgproc")
    public static native void dtFilter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, double d, double d2, int i, int i2);

    @Namespace("cv::ximgproc")
    public static native void fastBilateralSolverFilter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4);

    @Namespace("cv::ximgproc")
    public static native void fastBilateralSolverFilter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, double d, double d2, double d3, double d4, int i, double d5);

    @Namespace("cv::ximgproc")
    public static native void fastBilateralSolverFilter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4);

    @Namespace("cv::ximgproc")
    public static native void fastBilateralSolverFilter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, double d, double d2, double d3, double d4, int i, double d5);

    @Namespace("cv::ximgproc")
    public static native void fastBilateralSolverFilter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4);

    @Namespace("cv::ximgproc")
    public static native void fastBilateralSolverFilter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, double d, double d2, double d3, double d4, int i, double d5);

    @Namespace("cv::ximgproc")
    public static native void fastGlobalSmootherFilter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, double d, double d2);

    @Namespace("cv::ximgproc")
    public static native void fastGlobalSmootherFilter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, double d, double d2, double d3, int i);

    @Namespace("cv::ximgproc")
    public static native void fastGlobalSmootherFilter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, double d, double d2);

    @Namespace("cv::ximgproc")
    public static native void fastGlobalSmootherFilter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, double d, double d2, double d3, int i);

    @Namespace("cv::ximgproc")
    public static native void fastGlobalSmootherFilter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, double d, double d2);

    @Namespace("cv::ximgproc")
    public static native void fastGlobalSmootherFilter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, double d, double d2, double d3, int i);

    @Namespace("cv::ximgproc")
    public static native void getDisparityVis(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::ximgproc")
    public static native void getDisparityVis(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, double d);

    @Namespace("cv::ximgproc")
    public static native void getDisparityVis(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::ximgproc")
    public static native void getDisparityVis(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, double d);

    @Namespace("cv::ximgproc")
    public static native void getDisparityVis(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::ximgproc")
    public static native void getDisparityVis(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, double d);

    @Namespace("cv::ximgproc")
    public static native void guidedFilter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, int i, double d);

    @Namespace("cv::ximgproc")
    public static native void guidedFilter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, int i, double d, int i2);

    @Namespace("cv::ximgproc")
    public static native void guidedFilter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, int i, double d);

    @Namespace("cv::ximgproc")
    public static native void guidedFilter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, int i, double d, int i2);

    @Namespace("cv::ximgproc")
    public static native void guidedFilter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, int i, double d);

    @Namespace("cv::ximgproc")
    public static native void guidedFilter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, int i, double d, int i2);

    @Namespace("cv::ximgproc")
    public static native void jointBilateralFilter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, int i, double d, double d2);

    @Namespace("cv::ximgproc")
    public static native void jointBilateralFilter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, int i, double d, double d2, int i2);

    @Namespace("cv::ximgproc")
    public static native void jointBilateralFilter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, int i, double d, double d2);

    @Namespace("cv::ximgproc")
    public static native void jointBilateralFilter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, int i, double d, double d2, int i2);

    @Namespace("cv::ximgproc")
    public static native void jointBilateralFilter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, int i, double d, double d2);

    @Namespace("cv::ximgproc")
    public static native void jointBilateralFilter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, int i, double d, double d2, int i2);

    @Namespace("cv::ximgproc")
    public static native void l0Smooth(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::ximgproc")
    public static native void l0Smooth(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, double d, double d2);

    @Namespace("cv::ximgproc")
    public static native void l0Smooth(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::ximgproc")
    public static native void l0Smooth(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, double d, double d2);

    @Namespace("cv::ximgproc")
    public static native void l0Smooth(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::ximgproc")
    public static native void l0Smooth(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, double d, double d2);

    @Namespace("cv::ximgproc")
    public static native void niBlackThreshold(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, double d, int i, int i2, double d2);

    @Namespace("cv::ximgproc")
    public static native void niBlackThreshold(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, double d, int i, int i2, double d2, int i3);

    @Namespace("cv::ximgproc")
    public static native void niBlackThreshold(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, double d, int i, int i2, double d2);

    @Namespace("cv::ximgproc")
    public static native void niBlackThreshold(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, double d, int i, int i2, double d2, int i3);

    @Namespace("cv::ximgproc")
    public static native void niBlackThreshold(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, double d, int i, int i2, double d2);

    @Namespace("cv::ximgproc")
    public static native void niBlackThreshold(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, double d, int i, int i2, double d2, int i3);

    @Namespace("cv::ximgproc")
    public static native int readGT(@opencv_core.Str String str, @ByVal opencv_core.GpuMat gpuMat);

    @Namespace("cv::ximgproc")
    public static native int readGT(@opencv_core.Str String str, @ByVal opencv_core.Mat mat);

    @Namespace("cv::ximgproc")
    public static native int readGT(@opencv_core.Str String str, @ByVal opencv_core.UMat uMat);

    @Namespace("cv::ximgproc")
    public static native int readGT(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.GpuMat gpuMat);

    @Namespace("cv::ximgproc")
    public static native int readGT(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.Mat mat);

    @Namespace("cv::ximgproc")
    public static native int readGT(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.UMat uMat);

    @Namespace("cv::ximgproc")
    public static native void rollingGuidanceFilter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::ximgproc")
    public static native void rollingGuidanceFilter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, double d, double d2, int i2, int i3);

    @Namespace("cv::ximgproc")
    public static native void rollingGuidanceFilter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::ximgproc")
    public static native void rollingGuidanceFilter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, double d, double d2, int i2, int i3);

    @Namespace("cv::ximgproc")
    public static native void rollingGuidanceFilter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::ximgproc")
    public static native void rollingGuidanceFilter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, double d, double d2, int i2, int i3);

    @Namespace("cv::ximgproc")
    public static native void thinning(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::ximgproc")
    public static native void thinning(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i);

    @Namespace("cv::ximgproc")
    public static native void thinning(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::ximgproc")
    public static native void thinning(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i);

    @Namespace("cv::ximgproc")
    public static native void thinning(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::ximgproc")
    public static native void thinning(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i);

    static {
        Loader.load();
    }

    @Namespace("cv::ximgproc")
    public static class DTFilter extends opencv_core.Algorithm {
        public native void filter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void filter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i);

        public native void filter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void filter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i);

        public native void filter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void filter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i);

        static {
            Loader.load();
        }

        public DTFilter(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::ximgproc")
    public static class GuidedFilter extends opencv_core.Algorithm {
        public native void filter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void filter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i);

        public native void filter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void filter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i);

        public native void filter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void filter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i);

        static {
            Loader.load();
        }

        public GuidedFilter(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::ximgproc")
    public static class AdaptiveManifoldFilter extends opencv_core.Algorithm {
        @opencv_core.Ptr
        public static native AdaptiveManifoldFilter create();

        public native void collectGarbage();

        public native void filter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void filter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat3);

        public native void filter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void filter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat3);

        public native void filter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void filter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat3);

        @Cast({"bool"})
        public native boolean getAdjustOutliers();

        public native int getPCAIterations();

        public native double getSigmaR();

        public native double getSigmaS();

        public native int getTreeHeight();

        @Cast({"bool"})
        public native boolean getUseRNG();

        public native void setAdjustOutliers(@Cast({"bool"}) boolean z);

        public native void setPCAIterations(int i);

        public native void setSigmaR(double d);

        public native void setSigmaS(double d);

        public native void setTreeHeight(int i);

        public native void setUseRNG(@Cast({"bool"}) boolean z);

        static {
            Loader.load();
        }

        public AdaptiveManifoldFilter(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::ximgproc")
    public static class FastBilateralSolverFilter extends opencv_core.Algorithm {
        public native void filter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

        public native void filter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

        public native void filter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

        static {
            Loader.load();
        }

        public FastBilateralSolverFilter(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::ximgproc")
    public static class FastGlobalSmootherFilter extends opencv_core.Algorithm {
        public native void filter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void filter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void filter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        static {
            Loader.load();
        }

        public FastGlobalSmootherFilter(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::ximgproc")
    public static class DisparityFilter extends opencv_core.Algorithm {
        public native void filter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

        public native void filter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::Mat())") opencv_core.GpuMat gpuMat4, @ByVal(nullValue = "cv::Rect()") opencv_core.Rect rect, @ByVal(nullValue = "cv::InputArray(cv::Mat())") opencv_core.GpuMat gpuMat5);

        public native void filter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

        public native void filter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::Mat())") opencv_core.Mat mat4, @ByVal(nullValue = "cv::Rect()") opencv_core.Rect rect, @ByVal(nullValue = "cv::InputArray(cv::Mat())") opencv_core.Mat mat5);

        public native void filter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

        public native void filter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::Mat())") opencv_core.UMat uMat4, @ByVal(nullValue = "cv::Rect()") opencv_core.Rect rect, @ByVal(nullValue = "cv::InputArray(cv::Mat())") opencv_core.UMat uMat5);

        static {
            Loader.load();
        }

        public DisparityFilter(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::ximgproc")
    public static class DisparityWLSFilter extends DisparityFilter {
        @ByVal
        public native opencv_core.Mat getConfidenceMap();

        public native int getDepthDiscontinuityRadius();

        public native int getLRCthresh();

        public native double getLambda();

        @ByVal
        public native opencv_core.Rect getROI();

        public native double getSigmaColor();

        public native void setDepthDiscontinuityRadius(int i);

        public native void setLRCthresh(int i);

        public native void setLambda(double d);

        public native void setSigmaColor(double d);

        static {
            Loader.load();
        }

        public DisparityWLSFilter(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::ximgproc")
    public static class SparseMatchInterpolator extends opencv_core.Algorithm {
        public native void interpolate(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5);

        public native void interpolate(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5);

        public native void interpolate(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5);

        static {
            Loader.load();
        }

        public SparseMatchInterpolator(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::ximgproc")
    public static class EdgeAwareInterpolator extends SparseMatchInterpolator {
        public native float getFGSLambda();

        public native float getFGSSigma();

        public native int getK();

        public native float getLambda();

        public native float getSigma();

        @Cast({"bool"})
        public native boolean getUsePostProcessing();

        public native void setFGSLambda(float f);

        public native void setFGSSigma(float f);

        public native void setK(int i);

        public native void setLambda(float f);

        public native void setSigma(float f);

        public native void setUsePostProcessing(@Cast({"bool"}) boolean z);

        static {
            Loader.load();
        }

        public EdgeAwareInterpolator(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::ximgproc")
    public static class RFFeatureGetter extends opencv_core.Algorithm {
        public native void getFeatures(@ByRef @Const opencv_core.Mat mat, @ByRef opencv_core.Mat mat2, int i, int i2, int i3, int i4, int i5);

        static {
            Loader.load();
        }

        public RFFeatureGetter(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::ximgproc")
    public static class StructuredEdgeDetection extends opencv_core.Algorithm {
        public native void computeOrientation(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void computeOrientation(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void computeOrientation(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void detectEdges(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void detectEdges(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void detectEdges(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void edgesNms(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

        public native void edgesNms(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, int i, int i2, float f, @Cast({"bool"}) boolean z);

        public native void edgesNms(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

        public native void edgesNms(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, int i, int i2, float f, @Cast({"bool"}) boolean z);

        public native void edgesNms(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

        public native void edgesNms(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, int i, int i2, float f, @Cast({"bool"}) boolean z);

        static {
            Loader.load();
        }

        public StructuredEdgeDetection(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::ximgproc")
    public static class SuperpixelSEEDS extends opencv_core.Algorithm {
        public native void getLabelContourMask(@ByVal opencv_core.GpuMat gpuMat);

        public native void getLabelContourMask(@ByVal opencv_core.GpuMat gpuMat, @Cast({"bool"}) boolean z);

        public native void getLabelContourMask(@ByVal opencv_core.Mat mat);

        public native void getLabelContourMask(@ByVal opencv_core.Mat mat, @Cast({"bool"}) boolean z);

        public native void getLabelContourMask(@ByVal opencv_core.UMat uMat);

        public native void getLabelContourMask(@ByVal opencv_core.UMat uMat, @Cast({"bool"}) boolean z);

        public native void getLabels(@ByVal opencv_core.GpuMat gpuMat);

        public native void getLabels(@ByVal opencv_core.Mat mat);

        public native void getLabels(@ByVal opencv_core.UMat uMat);

        public native int getNumberOfSuperpixels();

        public native void iterate(@ByVal opencv_core.GpuMat gpuMat);

        public native void iterate(@ByVal opencv_core.GpuMat gpuMat, int i);

        public native void iterate(@ByVal opencv_core.Mat mat);

        public native void iterate(@ByVal opencv_core.Mat mat, int i);

        public native void iterate(@ByVal opencv_core.UMat uMat);

        public native void iterate(@ByVal opencv_core.UMat uMat, int i);

        static {
            Loader.load();
        }

        public SuperpixelSEEDS(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::ximgproc::segmentation")
    public static class GraphSegmentation extends opencv_core.Algorithm {
        public native float getK();

        public native int getMinSize();

        public native double getSigma();

        public native void processImage(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void processImage(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void processImage(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void setK(float f);

        public native void setMinSize(int i);

        public native void setSigma(double d);

        static {
            Loader.load();
        }

        public GraphSegmentation(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::ximgproc::segmentation")
    public static class SelectiveSearchSegmentationStrategy extends opencv_core.Algorithm {
        public native float get(int i, int i2);

        public native void merge(int i, int i2);

        public native void setImage(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

        public native void setImage(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, int i);

        public native void setImage(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

        public native void setImage(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, int i);

        public native void setImage(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

        public native void setImage(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, int i);

        static {
            Loader.load();
        }

        public SelectiveSearchSegmentationStrategy(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::ximgproc::segmentation")
    @Opaque
    public static class SelectiveSearchSegmentationStrategyColor extends SelectiveSearchSegmentationStrategy {
        public SelectiveSearchSegmentationStrategyColor() {
            super((Pointer) null);
        }

        public SelectiveSearchSegmentationStrategyColor(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::ximgproc::segmentation")
    @Opaque
    public static class SelectiveSearchSegmentationStrategySize extends SelectiveSearchSegmentationStrategy {
        public SelectiveSearchSegmentationStrategySize() {
            super((Pointer) null);
        }

        public SelectiveSearchSegmentationStrategySize(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::ximgproc::segmentation")
    @Opaque
    public static class SelectiveSearchSegmentationStrategyTexture extends SelectiveSearchSegmentationStrategy {
        public SelectiveSearchSegmentationStrategyTexture() {
            super((Pointer) null);
        }

        public SelectiveSearchSegmentationStrategyTexture(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::ximgproc::segmentation")
    @Opaque
    public static class SelectiveSearchSegmentationStrategyFill extends SelectiveSearchSegmentationStrategy {
        public SelectiveSearchSegmentationStrategyFill() {
            super((Pointer) null);
        }

        public SelectiveSearchSegmentationStrategyFill(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::ximgproc::segmentation")
    public static class SelectiveSearchSegmentationStrategyMultiple extends SelectiveSearchSegmentationStrategy {
        public native void addStrategy(@opencv_core.Ptr SelectiveSearchSegmentationStrategy selectiveSearchSegmentationStrategy, float f);

        public native void clearStrategies();

        static {
            Loader.load();
        }

        public SelectiveSearchSegmentationStrategyMultiple(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::ximgproc::segmentation")
    public static class SelectiveSearchSegmentation extends opencv_core.Algorithm {
        public native void addGraphSegmentation(@opencv_core.Ptr GraphSegmentation graphSegmentation);

        public native void addImage(@ByVal opencv_core.GpuMat gpuMat);

        public native void addImage(@ByVal opencv_core.Mat mat);

        public native void addImage(@ByVal opencv_core.UMat uMat);

        public native void addStrategy(@opencv_core.Ptr SelectiveSearchSegmentationStrategy selectiveSearchSegmentationStrategy);

        public native void clearGraphSegmentations();

        public native void clearImages();

        public native void clearStrategies();

        public native void process(@ByRef opencv_core.RectVector rectVector);

        public native void setBaseImage(@ByVal opencv_core.GpuMat gpuMat);

        public native void setBaseImage(@ByVal opencv_core.Mat mat);

        public native void setBaseImage(@ByVal opencv_core.UMat uMat);

        public native void switchToSelectiveSearchFast();

        public native void switchToSelectiveSearchFast(int i, int i2, float f);

        public native void switchToSelectiveSearchQuality();

        public native void switchToSelectiveSearchQuality(int i, int i2, float f);

        public native void switchToSingleStrategy();

        public native void switchToSingleStrategy(int i, float f);

        static {
            Loader.load();
        }

        public SelectiveSearchSegmentation(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::ximgproc")
    public static class SuperpixelSLIC extends opencv_core.Algorithm {
        public native void enforceLabelConnectivity();

        public native void enforceLabelConnectivity(int i);

        public native void getLabelContourMask(@ByVal opencv_core.GpuMat gpuMat);

        public native void getLabelContourMask(@ByVal opencv_core.GpuMat gpuMat, @Cast({"bool"}) boolean z);

        public native void getLabelContourMask(@ByVal opencv_core.Mat mat);

        public native void getLabelContourMask(@ByVal opencv_core.Mat mat, @Cast({"bool"}) boolean z);

        public native void getLabelContourMask(@ByVal opencv_core.UMat uMat);

        public native void getLabelContourMask(@ByVal opencv_core.UMat uMat, @Cast({"bool"}) boolean z);

        public native void getLabels(@ByVal opencv_core.GpuMat gpuMat);

        public native void getLabels(@ByVal opencv_core.Mat mat);

        public native void getLabels(@ByVal opencv_core.UMat uMat);

        public native int getNumberOfSuperpixels();

        public native void iterate();

        public native void iterate(int i);

        static {
            Loader.load();
        }

        public SuperpixelSLIC(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::ximgproc")
    public static class SuperpixelLSC extends opencv_core.Algorithm {
        public native void enforceLabelConnectivity();

        public native void enforceLabelConnectivity(int i);

        public native void getLabelContourMask(@ByVal opencv_core.GpuMat gpuMat);

        public native void getLabelContourMask(@ByVal opencv_core.GpuMat gpuMat, @Cast({"bool"}) boolean z);

        public native void getLabelContourMask(@ByVal opencv_core.Mat mat);

        public native void getLabelContourMask(@ByVal opencv_core.Mat mat, @Cast({"bool"}) boolean z);

        public native void getLabelContourMask(@ByVal opencv_core.UMat uMat);

        public native void getLabelContourMask(@ByVal opencv_core.UMat uMat, @Cast({"bool"}) boolean z);

        public native void getLabels(@ByVal opencv_core.GpuMat gpuMat);

        public native void getLabels(@ByVal opencv_core.Mat mat);

        public native void getLabels(@ByVal opencv_core.UMat uMat);

        public native int getNumberOfSuperpixels();

        public native void iterate();

        public native void iterate(int i);

        static {
            Loader.load();
        }

        public SuperpixelLSC(Pointer p) {
            super(p);
        }
    }
}
