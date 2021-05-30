package org.bytedeco.javacpp;

import java.nio.IntBuffer;
import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Const;
import org.bytedeco.javacpp.annotation.Name;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.opencv_imgproc;
import org.bytedeco.javacpp.presets.opencv_core;

public class opencv_cudaimgproc extends org.bytedeco.javacpp.presets.opencv_cudaimgproc {
    public static final int ALPHA_ATOP = 3;
    public static final int ALPHA_ATOP_PREMUL = 9;
    public static final int ALPHA_IN = 1;
    public static final int ALPHA_IN_PREMUL = 7;
    public static final int ALPHA_OUT = 2;
    public static final int ALPHA_OUT_PREMUL = 8;
    public static final int ALPHA_OVER = 0;
    public static final int ALPHA_OVER_PREMUL = 6;
    public static final int ALPHA_PLUS = 5;
    public static final int ALPHA_PLUS_PREMUL = 11;
    public static final int ALPHA_PREMUL = 12;
    public static final int ALPHA_XOR = 4;
    public static final int ALPHA_XOR_PREMUL = 10;
    public static final int COLOR_BayerBG2BGR_MHT = 256;
    public static final int COLOR_BayerBG2GRAY_MHT = 260;
    public static final int COLOR_BayerBG2RGB_MHT = 258;
    public static final int COLOR_BayerGB2BGR_MHT = 257;
    public static final int COLOR_BayerGB2GRAY_MHT = 261;
    public static final int COLOR_BayerGB2RGB_MHT = 259;
    public static final int COLOR_BayerGR2BGR_MHT = 259;
    public static final int COLOR_BayerGR2GRAY_MHT = 263;
    public static final int COLOR_BayerGR2RGB_MHT = 257;
    public static final int COLOR_BayerRG2BGR_MHT = 258;
    public static final int COLOR_BayerRG2GRAY_MHT = 262;
    public static final int COLOR_BayerRG2RGB_MHT = 256;

    @Namespace("cv::cuda")
    public static native void alphaComp(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, int i);

    @Namespace("cv::cuda")
    public static native void alphaComp(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void alphaComp(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, int i);

    @Namespace("cv::cuda")
    public static native void alphaComp(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void alphaComp(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, int i);

    @Namespace("cv::cuda")
    public static native void alphaComp(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void bilateralFilter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, float f, float f2);

    @Namespace("cv::cuda")
    public static native void bilateralFilter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, float f, float f2, int i2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void bilateralFilter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, float f, float f2);

    @Namespace("cv::cuda")
    public static native void bilateralFilter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, float f, float f2, int i2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void bilateralFilter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, float f, float f2);

    @Namespace("cv::cuda")
    public static native void bilateralFilter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, float f, float f2, int i2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void blendLinear(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5);

    @Namespace("cv::cuda")
    public static native void blendLinear(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void blendLinear(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5);

    @Namespace("cv::cuda")
    public static native void blendLinear(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void blendLinear(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5);

    @Namespace("cv::cuda")
    public static native void blendLinear(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void calcHist(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void calcHist(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv::cuda")
    public static native void calcHist(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void calcHist(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void calcHist(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void calcHist(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv::cuda")
    public static native void calcHist(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void calcHist(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void calcHist(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void calcHist(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void calcHist(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv::cuda")
    public static native void calcHist(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native CudaCLAHE createCLAHE();

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native CudaCLAHE createCLAHE(double d, @ByVal(nullValue = "cv::Size(8, 8)") opencv_core.Size size);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native CannyEdgeDetector createCannyEdgeDetector(double d, double d2);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native CannyEdgeDetector createCannyEdgeDetector(double d, double d2, int i, @Cast({"bool"}) boolean z);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native opencv_imgproc.GeneralizedHoughBallard createGeneralizedHoughBallard();

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native opencv_imgproc.GeneralizedHoughGuil createGeneralizedHoughGuil();

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native CornersDetector createGoodFeaturesToTrackDetector(int i);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native CornersDetector createGoodFeaturesToTrackDetector(int i, int i2, double d, double d2, int i3, @Cast({"bool"}) boolean z, double d3);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native CornernessCriteria createHarrisCorner(int i, int i2, int i3, double d);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native CornernessCriteria createHarrisCorner(int i, int i2, int i3, double d, int i4);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native HoughCirclesDetector createHoughCirclesDetector(float f, float f2, int i, int i2, int i3, int i4);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native HoughCirclesDetector createHoughCirclesDetector(float f, float f2, int i, int i2, int i3, int i4, int i5);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native HoughLinesDetector createHoughLinesDetector(float f, float f2, int i);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native HoughLinesDetector createHoughLinesDetector(float f, float f2, int i, @Cast({"bool"}) boolean z, int i2);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native HoughSegmentDetector createHoughSegmentDetector(float f, float f2, int i, int i2);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native HoughSegmentDetector createHoughSegmentDetector(float f, float f2, int i, int i2, int i3);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native CornernessCriteria createMinEigenValCorner(int i, int i2, int i3);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native CornernessCriteria createMinEigenValCorner(int i, int i2, int i3, int i4);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native TemplateMatching createTemplateMatching(int i, int i2);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native TemplateMatching createTemplateMatching(int i, int i2, @ByVal(nullValue = "cv::Size()") opencv_core.Size size);

    @Namespace("cv::cuda")
    public static native void cvtColor(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i);

    @Namespace("cv::cuda")
    public static native void cvtColor(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, int i2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void cvtColor(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i);

    @Namespace("cv::cuda")
    public static native void cvtColor(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, int i2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void cvtColor(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i);

    @Namespace("cv::cuda")
    public static native void cvtColor(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, int i2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void demosaicing(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i);

    @Namespace("cv::cuda")
    public static native void demosaicing(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, int i2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void demosaicing(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i);

    @Namespace("cv::cuda")
    public static native void demosaicing(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, int i2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void demosaicing(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i);

    @Namespace("cv::cuda")
    public static native void demosaicing(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, int i2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void equalizeHist(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void equalizeHist(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void equalizeHist(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void equalizeHist(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void equalizeHist(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void equalizeHist(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void evenLevels(@ByVal opencv_core.GpuMat gpuMat, int i, int i2, int i3);

    @Namespace("cv::cuda")
    public static native void evenLevels(@ByVal opencv_core.GpuMat gpuMat, int i, int i2, int i3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void evenLevels(@ByVal opencv_core.Mat mat, int i, int i2, int i3);

    @Namespace("cv::cuda")
    public static native void evenLevels(@ByVal opencv_core.Mat mat, int i, int i2, int i3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void evenLevels(@ByVal opencv_core.UMat uMat, int i, int i2, int i3);

    @Namespace("cv::cuda")
    public static native void evenLevels(@ByVal opencv_core.UMat uMat, int i, int i2, int i3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void gammaCorrection(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void gammaCorrection(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @Cast({"bool"}) boolean z, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void gammaCorrection(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void gammaCorrection(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @Cast({"bool"}) boolean z, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void gammaCorrection(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void gammaCorrection(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @Cast({"bool"}) boolean z, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void histEven(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, int i2, int i3);

    @Namespace("cv::cuda")
    public static native void histEven(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, int i2, int i3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void histEven(@ByVal opencv_core.GpuMat gpuMat, opencv_core.GpuMat gpuMat2, IntBuffer intBuffer, IntBuffer intBuffer2, IntBuffer intBuffer3);

    @Namespace("cv::cuda")
    public static native void histEven(@ByVal opencv_core.GpuMat gpuMat, opencv_core.GpuMat gpuMat2, IntBuffer intBuffer, IntBuffer intBuffer2, IntBuffer intBuffer3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void histEven(@ByVal opencv_core.GpuMat gpuMat, opencv_core.GpuMat gpuMat2, IntPointer intPointer, IntPointer intPointer2, IntPointer intPointer3);

    @Namespace("cv::cuda")
    public static native void histEven(@ByVal opencv_core.GpuMat gpuMat, opencv_core.GpuMat gpuMat2, IntPointer intPointer, IntPointer intPointer2, IntPointer intPointer3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void histEven(@ByVal opencv_core.GpuMat gpuMat, opencv_core.GpuMat gpuMat2, int[] iArr, int[] iArr2, int[] iArr3);

    @Namespace("cv::cuda")
    public static native void histEven(@ByVal opencv_core.GpuMat gpuMat, opencv_core.GpuMat gpuMat2, int[] iArr, int[] iArr2, int[] iArr3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void histEven(@ByVal opencv_core.Mat mat, opencv_core.GpuMat gpuMat, IntBuffer intBuffer, IntBuffer intBuffer2, IntBuffer intBuffer3);

    @Namespace("cv::cuda")
    public static native void histEven(@ByVal opencv_core.Mat mat, opencv_core.GpuMat gpuMat, IntBuffer intBuffer, IntBuffer intBuffer2, IntBuffer intBuffer3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void histEven(@ByVal opencv_core.Mat mat, opencv_core.GpuMat gpuMat, IntPointer intPointer, IntPointer intPointer2, IntPointer intPointer3);

    @Namespace("cv::cuda")
    public static native void histEven(@ByVal opencv_core.Mat mat, opencv_core.GpuMat gpuMat, IntPointer intPointer, IntPointer intPointer2, IntPointer intPointer3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void histEven(@ByVal opencv_core.Mat mat, opencv_core.GpuMat gpuMat, int[] iArr, int[] iArr2, int[] iArr3);

    @Namespace("cv::cuda")
    public static native void histEven(@ByVal opencv_core.Mat mat, opencv_core.GpuMat gpuMat, int[] iArr, int[] iArr2, int[] iArr3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void histEven(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, int i2, int i3);

    @Namespace("cv::cuda")
    public static native void histEven(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, int i2, int i3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void histEven(@ByVal opencv_core.UMat uMat, opencv_core.GpuMat gpuMat, IntBuffer intBuffer, IntBuffer intBuffer2, IntBuffer intBuffer3);

    @Namespace("cv::cuda")
    public static native void histEven(@ByVal opencv_core.UMat uMat, opencv_core.GpuMat gpuMat, IntBuffer intBuffer, IntBuffer intBuffer2, IntBuffer intBuffer3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void histEven(@ByVal opencv_core.UMat uMat, opencv_core.GpuMat gpuMat, IntPointer intPointer, IntPointer intPointer2, IntPointer intPointer3);

    @Namespace("cv::cuda")
    public static native void histEven(@ByVal opencv_core.UMat uMat, opencv_core.GpuMat gpuMat, IntPointer intPointer, IntPointer intPointer2, IntPointer intPointer3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void histEven(@ByVal opencv_core.UMat uMat, opencv_core.GpuMat gpuMat, int[] iArr, int[] iArr2, int[] iArr3);

    @Namespace("cv::cuda")
    public static native void histEven(@ByVal opencv_core.UMat uMat, opencv_core.GpuMat gpuMat, int[] iArr, int[] iArr2, int[] iArr3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void histEven(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, int i2, int i3);

    @Namespace("cv::cuda")
    public static native void histEven(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, int i2, int i3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void histRange(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv::cuda")
    public static native void histRange(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void histRange(@ByVal opencv_core.Mat mat, opencv_core.GpuMat gpuMat, @Const opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void histRange(@ByVal opencv_core.Mat mat, opencv_core.GpuMat gpuMat, @Const opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void histRange(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv::cuda")
    public static native void histRange(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void histRange(@ByVal opencv_core.UMat uMat, opencv_core.GpuMat gpuMat, @Const opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void histRange(@ByVal opencv_core.UMat uMat, opencv_core.GpuMat gpuMat, @Const opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void histRange(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv::cuda")
    public static native void histRange(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void meanShiftFiltering(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, int i2);

    @Namespace("cv::cuda")
    public static native void meanShiftFiltering(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, int i2, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 5, 1)") opencv_core.TermCriteria termCriteria, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void meanShiftFiltering(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, int i2);

    @Namespace("cv::cuda")
    public static native void meanShiftFiltering(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, int i2, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 5, 1)") opencv_core.TermCriteria termCriteria, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void meanShiftFiltering(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, int i2);

    @Namespace("cv::cuda")
    public static native void meanShiftFiltering(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, int i2, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 5, 1)") opencv_core.TermCriteria termCriteria, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void meanShiftProc(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, int i, int i2);

    @Namespace("cv::cuda")
    public static native void meanShiftProc(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, int i, int i2, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 5, 1)") opencv_core.TermCriteria termCriteria, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void meanShiftProc(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, int i, int i2);

    @Namespace("cv::cuda")
    public static native void meanShiftProc(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, int i, int i2, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 5, 1)") opencv_core.TermCriteria termCriteria, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void meanShiftProc(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, int i, int i2);

    @Namespace("cv::cuda")
    public static native void meanShiftProc(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, int i, int i2, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 5, 1)") opencv_core.TermCriteria termCriteria, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void meanShiftSegmentation(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, int i2, int i3);

    @Namespace("cv::cuda")
    public static native void meanShiftSegmentation(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, int i2, int i3, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 5, 1)") opencv_core.TermCriteria termCriteria, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void meanShiftSegmentation(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, int i2, int i3);

    @Namespace("cv::cuda")
    public static native void meanShiftSegmentation(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, int i2, int i3, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 5, 1)") opencv_core.TermCriteria termCriteria, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void meanShiftSegmentation(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, int i2, int i3);

    @Namespace("cv::cuda")
    public static native void meanShiftSegmentation(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, int i2, int i3, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 5, 1)") opencv_core.TermCriteria termCriteria, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void swapChannels(@ByVal opencv_core.GpuMat gpuMat, @Const IntBuffer intBuffer);

    @Namespace("cv::cuda")
    public static native void swapChannels(@ByVal opencv_core.GpuMat gpuMat, @Const IntBuffer intBuffer, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void swapChannels(@ByVal opencv_core.GpuMat gpuMat, @Const IntPointer intPointer);

    @Namespace("cv::cuda")
    public static native void swapChannels(@ByVal opencv_core.GpuMat gpuMat, @Const IntPointer intPointer, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void swapChannels(@ByVal opencv_core.GpuMat gpuMat, @Const int[] iArr);

    @Namespace("cv::cuda")
    public static native void swapChannels(@ByVal opencv_core.GpuMat gpuMat, @Const int[] iArr, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void swapChannels(@ByVal opencv_core.Mat mat, @Const IntBuffer intBuffer);

    @Namespace("cv::cuda")
    public static native void swapChannels(@ByVal opencv_core.Mat mat, @Const IntBuffer intBuffer, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void swapChannels(@ByVal opencv_core.Mat mat, @Const IntPointer intPointer);

    @Namespace("cv::cuda")
    public static native void swapChannels(@ByVal opencv_core.Mat mat, @Const IntPointer intPointer, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void swapChannels(@ByVal opencv_core.Mat mat, @Const int[] iArr);

    @Namespace("cv::cuda")
    public static native void swapChannels(@ByVal opencv_core.Mat mat, @Const int[] iArr, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void swapChannels(@ByVal opencv_core.UMat uMat, @Const IntBuffer intBuffer);

    @Namespace("cv::cuda")
    public static native void swapChannels(@ByVal opencv_core.UMat uMat, @Const IntBuffer intBuffer, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void swapChannels(@ByVal opencv_core.UMat uMat, @Const IntPointer intPointer);

    @Namespace("cv::cuda")
    public static native void swapChannels(@ByVal opencv_core.UMat uMat, @Const IntPointer intPointer, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void swapChannels(@ByVal opencv_core.UMat uMat, @Const int[] iArr);

    @Namespace("cv::cuda")
    public static native void swapChannels(@ByVal opencv_core.UMat uMat, @Const int[] iArr, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    static {
        Loader.load();
    }

    @Name({"cv::cuda::CLAHE"})
    public static class CudaCLAHE extends opencv_imgproc.CLAHE {
        public native void apply(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef opencv_core.Stream stream);

        public native void apply(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef opencv_core.Stream stream);

        public native void apply(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef opencv_core.Stream stream);

        static {
            Loader.load();
        }

        public CudaCLAHE(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::cuda")
    public static class CannyEdgeDetector extends opencv_core.Algorithm {
        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void detect(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void detect(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

        public native void detect(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void detect(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native int getAppertureSize();

        public native double getHighThreshold();

        @Cast({"bool"})
        public native boolean getL2Gradient();

        public native double getLowThreshold();

        public native void setAppertureSize(int i);

        public native void setHighThreshold(double d);

        public native void setL2Gradient(@Cast({"bool"}) boolean z);

        public native void setLowThreshold(double d);

        static {
            Loader.load();
        }

        public CannyEdgeDetector(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::cuda")
    public static class HoughLinesDetector extends opencv_core.Algorithm {
        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void detect(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void detect(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void downloadResults(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void downloadResults(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void downloadResults(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void downloadResults(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void downloadResults(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void downloadResults(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        @Cast({"bool"})
        public native boolean getDoSort();

        public native int getMaxLines();

        public native float getRho();

        public native float getTheta();

        public native int getThreshold();

        public native void setDoSort(@Cast({"bool"}) boolean z);

        public native void setMaxLines(int i);

        public native void setRho(float f);

        public native void setTheta(float f);

        public native void setThreshold(int i);

        static {
            Loader.load();
        }

        public HoughLinesDetector(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::cuda")
    public static class HoughSegmentDetector extends opencv_core.Algorithm {
        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void detect(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void detect(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native int getMaxLineGap();

        public native int getMaxLines();

        public native int getMinLineLength();

        public native float getRho();

        public native float getTheta();

        public native void setMaxLineGap(int i);

        public native void setMaxLines(int i);

        public native void setMinLineLength(int i);

        public native void setRho(float f);

        public native void setTheta(float f);

        static {
            Loader.load();
        }

        public HoughSegmentDetector(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::cuda")
    public static class HoughCirclesDetector extends opencv_core.Algorithm {
        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void detect(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void detect(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native int getCannyThreshold();

        public native float getDp();

        public native int getMaxCircles();

        public native int getMaxRadius();

        public native float getMinDist();

        public native int getMinRadius();

        public native int getVotesThreshold();

        public native void setCannyThreshold(int i);

        public native void setDp(float f);

        public native void setMaxCircles(int i);

        public native void setMaxRadius(int i);

        public native void setMinDist(float f);

        public native void setMinRadius(int i);

        public native void setVotesThreshold(int i);

        static {
            Loader.load();
        }

        public HoughCirclesDetector(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::cuda")
    public static class CornernessCriteria extends opencv_core.Algorithm {
        public native void compute(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void compute(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void compute(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void compute(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void compute(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void compute(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        static {
            Loader.load();
        }

        public CornernessCriteria(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::cuda")
    public static class CornersDetector extends opencv_core.Algorithm {
        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void detect(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void detect(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        static {
            Loader.load();
        }

        public CornersDetector(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::cuda")
    public static class TemplateMatching extends opencv_core.Algorithm {
        public native void match(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

        public native void match(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void match(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

        public native void match(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void match(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

        public native void match(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        static {
            Loader.load();
        }

        public TemplateMatching(Pointer p) {
            super(p);
        }
    }
}
