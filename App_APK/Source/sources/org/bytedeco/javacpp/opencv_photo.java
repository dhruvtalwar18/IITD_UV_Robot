package org.bytedeco.javacpp;

import java.nio.FloatBuffer;
import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Const;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.annotation.StdVector;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.presets.opencv_core;

public class opencv_photo extends org.bytedeco.javacpp.presets.opencv_photo {
    public static final int INPAINT_NS = 0;
    public static final int INPAINT_TELEA = 1;
    public static final int LDR_SIZE = 256;
    public static final int MIXED_CLONE = 2;
    public static final int MONOCHROME_TRANSFER = 3;
    public static final int NORMAL_CLONE = 1;
    public static final int NORMCONV_FILTER = 2;
    public static final int RECURS_FILTER = 1;

    @Namespace("cv")
    public static native void colorChange(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv")
    public static native void colorChange(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, float f, float f2, float f3);

    @Namespace("cv")
    public static native void colorChange(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv")
    public static native void colorChange(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, float f, float f2, float f3);

    @Namespace("cv")
    public static native void colorChange(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv")
    public static native void colorChange(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, float f, float f2, float f3);

    @Namespace("cv")
    @opencv_core.Ptr
    public static native AlignMTB createAlignMTB();

    @Namespace("cv")
    @opencv_core.Ptr
    public static native AlignMTB createAlignMTB(int i, int i2, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    @opencv_core.Ptr
    public static native CalibrateDebevec createCalibrateDebevec();

    @Namespace("cv")
    @opencv_core.Ptr
    public static native CalibrateDebevec createCalibrateDebevec(int i, float f, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    @opencv_core.Ptr
    public static native CalibrateRobertson createCalibrateRobertson();

    @Namespace("cv")
    @opencv_core.Ptr
    public static native CalibrateRobertson createCalibrateRobertson(int i, float f);

    @Namespace("cv")
    @opencv_core.Ptr
    public static native MergeDebevec createMergeDebevec();

    @Namespace("cv")
    @opencv_core.Ptr
    public static native MergeMertens createMergeMertens();

    @Namespace("cv")
    @opencv_core.Ptr
    public static native MergeMertens createMergeMertens(float f, float f2, float f3);

    @Namespace("cv")
    @opencv_core.Ptr
    public static native MergeRobertson createMergeRobertson();

    @Namespace("cv")
    @opencv_core.Ptr
    public static native Tonemap createTonemap();

    @Namespace("cv")
    @opencv_core.Ptr
    public static native Tonemap createTonemap(float f);

    @Namespace("cv")
    @opencv_core.Ptr
    public static native TonemapDrago createTonemapDrago();

    @Namespace("cv")
    @opencv_core.Ptr
    public static native TonemapDrago createTonemapDrago(float f, float f2, float f3);

    @Namespace("cv")
    @opencv_core.Ptr
    public static native TonemapMantiuk createTonemapMantiuk();

    @Namespace("cv")
    @opencv_core.Ptr
    public static native TonemapMantiuk createTonemapMantiuk(float f, float f2, float f3);

    @Namespace("cv")
    @opencv_core.Ptr
    public static native TonemapReinhard createTonemapReinhard();

    @Namespace("cv")
    @opencv_core.Ptr
    public static native TonemapReinhard createTonemapReinhard(float f, float f2, float f3, float f4);

    @Namespace("cv")
    public static native void decolor(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv")
    public static native void decolor(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv")
    public static native void decolor(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv")
    public static native void denoise_TVL1(@ByRef @Const opencv_core.MatVector matVector, @ByRef opencv_core.Mat mat);

    @Namespace("cv")
    public static native void denoise_TVL1(@ByRef @Const opencv_core.MatVector matVector, @ByRef opencv_core.Mat mat, double d, int i);

    @Namespace("cv")
    public static native void detailEnhance(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv")
    public static native void detailEnhance(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, float f, float f2);

    @Namespace("cv")
    public static native void detailEnhance(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv")
    public static native void detailEnhance(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, float f, float f2);

    @Namespace("cv")
    public static native void detailEnhance(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv")
    public static native void detailEnhance(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, float f, float f2);

    @Namespace("cv")
    public static native void edgePreservingFilter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv")
    public static native void edgePreservingFilter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, float f, float f2);

    @Namespace("cv")
    public static native void edgePreservingFilter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv")
    public static native void edgePreservingFilter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, float f, float f2);

    @Namespace("cv")
    public static native void edgePreservingFilter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv")
    public static native void edgePreservingFilter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, float f, float f2);

    @Namespace("cv")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, float f);

    @Namespace("cv")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, float f, int i, int i2);

    @Namespace("cv::cuda")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, float f, int i, int i2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @StdVector FloatBuffer floatBuffer);

    @Namespace("cv")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @StdVector FloatBuffer floatBuffer, int i, int i2, int i3);

    @Namespace("cv")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @StdVector FloatPointer floatPointer);

    @Namespace("cv")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @StdVector FloatPointer floatPointer, int i, int i2, int i3);

    @Namespace("cv")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @StdVector float[] fArr);

    @Namespace("cv")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @StdVector float[] fArr, int i, int i2, int i3);

    @Namespace("cv")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, float f);

    @Namespace("cv")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, float f, int i, int i2);

    @Namespace("cv::cuda")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, float f, int i, int i2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @StdVector FloatBuffer floatBuffer);

    @Namespace("cv")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @StdVector FloatBuffer floatBuffer, int i, int i2, int i3);

    @Namespace("cv")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @StdVector FloatPointer floatPointer);

    @Namespace("cv")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @StdVector FloatPointer floatPointer, int i, int i2, int i3);

    @Namespace("cv")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @StdVector float[] fArr);

    @Namespace("cv")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @StdVector float[] fArr, int i, int i2, int i3);

    @Namespace("cv")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, float f);

    @Namespace("cv")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, float f, int i, int i2);

    @Namespace("cv::cuda")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, float f, int i, int i2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @StdVector FloatBuffer floatBuffer);

    @Namespace("cv")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @StdVector FloatBuffer floatBuffer, int i, int i2, int i3);

    @Namespace("cv")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @StdVector FloatPointer floatPointer);

    @Namespace("cv")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @StdVector FloatPointer floatPointer, int i, int i2, int i3);

    @Namespace("cv")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @StdVector float[] fArr);

    @Namespace("cv")
    public static native void fastNlMeansDenoising(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @StdVector float[] fArr, int i, int i2, int i3);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingColored(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void fastNlMeansDenoisingColored(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, float f, float f2);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingColored(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, float f, float f2, int i, int i2);

    @Namespace("cv::cuda")
    public static native void fastNlMeansDenoisingColored(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, float f, float f2, int i, int i2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingColored(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void fastNlMeansDenoisingColored(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, float f, float f2);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingColored(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, float f, float f2, int i, int i2);

    @Namespace("cv::cuda")
    public static native void fastNlMeansDenoisingColored(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, float f, float f2, int i, int i2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingColored(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void fastNlMeansDenoisingColored(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, float f, float f2);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingColored(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, float f, float f2, int i, int i2);

    @Namespace("cv::cuda")
    public static native void fastNlMeansDenoisingColored(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, float f, float f2, int i, int i2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingColoredMulti(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat, int i, int i2);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingColoredMulti(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat, int i, int i2, float f, float f2, int i3, int i4);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingColoredMulti(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat, int i, int i2);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingColoredMulti(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat, int i, int i2, float f, float f2, int i3, int i4);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingColoredMulti(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat, int i, int i2);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingColoredMulti(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat, int i, int i2, float f, float f2, int i3, int i4);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingColoredMulti(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, int i, int i2);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingColoredMulti(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, int i, int i2, float f, float f2, int i3, int i4);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingColoredMulti(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, int i, int i2);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingColoredMulti(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, int i, int i2, float f, float f2, int i3, int i4);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingColoredMulti(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, int i, int i2);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingColoredMulti(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, int i, int i2, float f, float f2, int i3, int i4);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingColoredMulti(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat, int i, int i2);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingColoredMulti(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat, int i, int i2, float f, float f2, int i3, int i4);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingColoredMulti(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat, int i, int i2);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingColoredMulti(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat, int i, int i2, float f, float f2, int i3, int i4);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingColoredMulti(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat, int i, int i2);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingColoredMulti(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat, int i, int i2, float f, float f2, int i3, int i4);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat, int i, int i2);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat, int i, int i2, float f, int i3, int i4);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat, int i, int i2, @StdVector float[] fArr);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat, int i, int i2, @StdVector float[] fArr, int i3, int i4, int i5);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat, int i, int i2);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat, int i, int i2, float f, int i3, int i4);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat, int i, int i2, @StdVector float[] fArr);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat, int i, int i2, @StdVector float[] fArr, int i3, int i4, int i5);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat, int i, int i2);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat, int i, int i2, float f, int i3, int i4);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat, int i, int i2, @StdVector float[] fArr);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat, int i, int i2, @StdVector float[] fArr, int i3, int i4, int i5);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, int i, int i2);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, int i, int i2, float f, int i3, int i4);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, int i, int i2, @StdVector FloatPointer floatPointer);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, int i, int i2, @StdVector FloatPointer floatPointer, int i3, int i4, int i5);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, int i, int i2);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, int i, int i2, float f, int i3, int i4);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, int i, int i2, @StdVector FloatPointer floatPointer);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, int i, int i2, @StdVector FloatPointer floatPointer, int i3, int i4, int i5);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, int i, int i2);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, int i, int i2, float f, int i3, int i4);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, int i, int i2, @StdVector FloatPointer floatPointer);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, int i, int i2, @StdVector FloatPointer floatPointer, int i3, int i4, int i5);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat, int i, int i2);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat, int i, int i2, float f, int i3, int i4);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat, int i, int i2, @StdVector FloatBuffer floatBuffer);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat, int i, int i2, @StdVector FloatBuffer floatBuffer, int i3, int i4, int i5);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat, int i, int i2);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat, int i, int i2, float f, int i3, int i4);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat, int i, int i2, @StdVector FloatBuffer floatBuffer);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat, int i, int i2, @StdVector FloatBuffer floatBuffer, int i3, int i4, int i5);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat, int i, int i2);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat, int i, int i2, float f, int i3, int i4);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat, int i, int i2, @StdVector FloatBuffer floatBuffer);

    @Namespace("cv")
    public static native void fastNlMeansDenoisingMulti(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat, int i, int i2, @StdVector FloatBuffer floatBuffer, int i3, int i4, int i5);

    @Namespace("cv")
    public static native void illuminationChange(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv")
    public static native void illuminationChange(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, float f, float f2);

    @Namespace("cv")
    public static native void illuminationChange(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv")
    public static native void illuminationChange(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, float f, float f2);

    @Namespace("cv")
    public static native void illuminationChange(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv")
    public static native void illuminationChange(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, float f, float f2);

    @Namespace("cv")
    public static native void inpaint(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, double d, int i);

    @Namespace("cv")
    public static native void inpaint(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, double d, int i);

    @Namespace("cv")
    public static native void inpaint(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, double d, int i);

    @Namespace("cv::cuda")
    public static native void nonLocalMeans(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, float f);

    @Namespace("cv::cuda")
    public static native void nonLocalMeans(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, float f, int i, int i2, int i3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void nonLocalMeans(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, float f);

    @Namespace("cv::cuda")
    public static native void nonLocalMeans(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, float f, int i, int i2, int i3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void nonLocalMeans(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, float f);

    @Namespace("cv::cuda")
    public static native void nonLocalMeans(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, float f, int i, int i2, int i3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv")
    public static native void pencilSketch(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv")
    public static native void pencilSketch(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, float f, float f2, float f3);

    @Namespace("cv")
    public static native void pencilSketch(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv")
    public static native void pencilSketch(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, float f, float f2, float f3);

    @Namespace("cv")
    public static native void pencilSketch(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv")
    public static native void pencilSketch(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, float f, float f2, float f3);

    @Namespace("cv")
    public static native void seamlessClone(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.Point point, @ByVal opencv_core.GpuMat gpuMat4, int i);

    @Namespace("cv")
    public static native void seamlessClone(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Point point, @ByVal opencv_core.Mat mat4, int i);

    @Namespace("cv")
    public static native void seamlessClone(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.Point point, @ByVal opencv_core.UMat uMat4, int i);

    @Namespace("cv")
    public static native void stylization(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv")
    public static native void stylization(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, float f, float f2);

    @Namespace("cv")
    public static native void stylization(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv")
    public static native void stylization(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, float f, float f2);

    @Namespace("cv")
    public static native void stylization(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv")
    public static native void stylization(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, float f, float f2);

    @Namespace("cv")
    public static native void textureFlattening(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv")
    public static native void textureFlattening(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, float f, float f2, int i);

    @Namespace("cv")
    public static native void textureFlattening(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv")
    public static native void textureFlattening(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, float f, float f2, int i);

    @Namespace("cv")
    public static native void textureFlattening(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv")
    public static native void textureFlattening(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, float f, float f2, int i);

    static {
        Loader.load();
    }

    @Namespace("cv")
    public static class Tonemap extends opencv_core.Algorithm {
        public native float getGamma();

        public native void process(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void process(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void process(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void setGamma(float f);

        static {
            Loader.load();
        }

        public Tonemap(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class TonemapDrago extends Tonemap {
        public native float getBias();

        public native float getSaturation();

        public native void setBias(float f);

        public native void setSaturation(float f);

        static {
            Loader.load();
        }

        public TonemapDrago(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class TonemapReinhard extends Tonemap {
        public native float getColorAdaptation();

        public native float getIntensity();

        public native float getLightAdaptation();

        public native void setColorAdaptation(float f);

        public native void setIntensity(float f);

        public native void setLightAdaptation(float f);

        static {
            Loader.load();
        }

        public TonemapReinhard(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class TonemapMantiuk extends Tonemap {
        public native float getSaturation();

        public native float getScale();

        public native void setSaturation(float f);

        public native void setScale(float f);

        static {
            Loader.load();
        }

        public TonemapMantiuk(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class AlignExposures extends opencv_core.Algorithm {
        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByRef opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByRef opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByRef opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByRef opencv_core.MatVector matVector2, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByRef opencv_core.MatVector matVector2, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByRef opencv_core.MatVector matVector2, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByRef opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByRef opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByRef opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        static {
            Loader.load();
        }

        public AlignExposures(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class AlignMTB extends AlignExposures {
        @ByVal
        public native opencv_core.Point calculateShift(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        @ByVal
        public native opencv_core.Point calculateShift(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        @ByVal
        public native opencv_core.Point calculateShift(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void computeBitmaps(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

        public native void computeBitmaps(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

        public native void computeBitmaps(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

        @Cast({"bool"})
        public native boolean getCut();

        public native int getExcludeRange();

        public native int getMaxBits();

        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByRef opencv_core.MatVector matVector);

        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByRef opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByRef opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByRef opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByRef opencv_core.MatVector matVector2);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByRef opencv_core.MatVector matVector2, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByRef opencv_core.MatVector matVector2, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByRef opencv_core.MatVector matVector2, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByRef opencv_core.MatVector matVector);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByRef opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByRef opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByRef opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void setCut(@Cast({"bool"}) boolean z);

        public native void setExcludeRange(int i);

        public native void setMaxBits(int i);

        public native void shiftMat(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @Const @ByVal opencv_core.Point point);

        public native void shiftMat(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @Const @ByVal opencv_core.Point point);

        public native void shiftMat(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @Const @ByVal opencv_core.Point point);

        static {
            Loader.load();
        }

        public AlignMTB(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class CalibrateCRF extends opencv_core.Algorithm {
        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        static {
            Loader.load();
        }

        public CalibrateCRF(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class CalibrateDebevec extends CalibrateCRF {
        public native float getLambda();

        @Cast({"bool"})
        public native boolean getRandom();

        public native int getSamples();

        public native void setLambda(float f);

        public native void setRandom(@Cast({"bool"}) boolean z);

        public native void setSamples(int i);

        static {
            Loader.load();
        }

        public CalibrateDebevec(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class CalibrateRobertson extends CalibrateCRF {
        public native int getMaxIter();

        @ByVal
        public native opencv_core.Mat getRadiance();

        public native float getThreshold();

        public native void setMaxIter(int i);

        public native void setThreshold(float f);

        static {
            Loader.load();
        }

        public CalibrateRobertson(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class MergeExposures extends opencv_core.Algorithm {
        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

        static {
            Loader.load();
        }

        public MergeExposures(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class MergeDebevec extends MergeExposures {
        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

        static {
            Loader.load();
        }

        public MergeDebevec(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class MergeMertens extends MergeExposures {
        public native float getContrastWeight();

        public native float getExposureWeight();

        public native float getSaturationWeight();

        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat);

        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat);

        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat);

        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

        public native void setContrastWeight(float f);

        public native void setExposureWeight(float f);

        public native void setSaturationWeight(float f);

        static {
            Loader.load();
        }

        public MergeMertens(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class MergeRobertson extends MergeExposures {
        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void process(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void process(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void process(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

        static {
            Loader.load();
        }

        public MergeRobertson(Pointer p) {
            super(p);
        }
    }
}
