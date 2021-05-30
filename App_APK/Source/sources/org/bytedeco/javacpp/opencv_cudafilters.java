package org.bytedeco.javacpp;

import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.presets.opencv_core;

public class opencv_cudafilters extends org.bytedeco.javacpp.presets.opencv_cudafilters {
    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createBoxFilter(int i, int i2, @ByVal opencv_core.Size size);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createBoxFilter(int i, int i2, @ByVal opencv_core.Size size, @ByVal(nullValue = "cv::Point(-1, -1)") opencv_core.Point point, int i3, @ByVal(nullValue = "cv::Scalar::all(0)") opencv_core.Scalar scalar);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createBoxMaxFilter(int i, @ByVal opencv_core.Size size);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createBoxMaxFilter(int i, @ByVal opencv_core.Size size, @ByVal(nullValue = "cv::Point(-1, -1)") opencv_core.Point point, int i2, @ByVal(nullValue = "cv::Scalar::all(0)") opencv_core.Scalar scalar);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createBoxMinFilter(int i, @ByVal opencv_core.Size size);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createBoxMinFilter(int i, @ByVal opencv_core.Size size, @ByVal(nullValue = "cv::Point(-1, -1)") opencv_core.Point point, int i2, @ByVal(nullValue = "cv::Scalar::all(0)") opencv_core.Scalar scalar);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createColumnSumFilter(int i, int i2, int i3);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createColumnSumFilter(int i, int i2, int i3, int i4, int i5, @ByVal(nullValue = "cv::Scalar::all(0)") opencv_core.Scalar scalar);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createDerivFilter(int i, int i2, int i3, int i4, int i5);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createDerivFilter(int i, int i2, int i3, int i4, int i5, @Cast({"bool"}) boolean z, double d, int i6, int i7);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createGaussianFilter(int i, int i2, @ByVal opencv_core.Size size, double d);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createGaussianFilter(int i, int i2, @ByVal opencv_core.Size size, double d, double d2, int i3, int i4);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createLaplacianFilter(int i, int i2);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createLaplacianFilter(int i, int i2, int i3, double d, int i4, @ByVal(nullValue = "cv::Scalar::all(0)") opencv_core.Scalar scalar);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createLinearFilter(int i, int i2, @ByVal opencv_core.GpuMat gpuMat);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createLinearFilter(int i, int i2, @ByVal opencv_core.GpuMat gpuMat, @ByVal(nullValue = "cv::Point(-1, -1)") opencv_core.Point point, int i3, @ByVal(nullValue = "cv::Scalar::all(0)") opencv_core.Scalar scalar);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createLinearFilter(int i, int i2, @ByVal opencv_core.Mat mat);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createLinearFilter(int i, int i2, @ByVal opencv_core.Mat mat, @ByVal(nullValue = "cv::Point(-1, -1)") opencv_core.Point point, int i3, @ByVal(nullValue = "cv::Scalar::all(0)") opencv_core.Scalar scalar);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createLinearFilter(int i, int i2, @ByVal opencv_core.UMat uMat);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createLinearFilter(int i, int i2, @ByVal opencv_core.UMat uMat, @ByVal(nullValue = "cv::Point(-1, -1)") opencv_core.Point point, int i3, @ByVal(nullValue = "cv::Scalar::all(0)") opencv_core.Scalar scalar);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createMedianFilter(int i, int i2);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createMedianFilter(int i, int i2, int i3);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createMorphologyFilter(int i, int i2, @ByVal opencv_core.GpuMat gpuMat);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createMorphologyFilter(int i, int i2, @ByVal opencv_core.GpuMat gpuMat, @ByVal(nullValue = "cv::Point(-1, -1)") opencv_core.Point point, int i3);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createMorphologyFilter(int i, int i2, @ByVal opencv_core.Mat mat);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createMorphologyFilter(int i, int i2, @ByVal opencv_core.Mat mat, @ByVal(nullValue = "cv::Point(-1, -1)") opencv_core.Point point, int i3);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createMorphologyFilter(int i, int i2, @ByVal opencv_core.UMat uMat);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createMorphologyFilter(int i, int i2, @ByVal opencv_core.UMat uMat, @ByVal(nullValue = "cv::Point(-1, -1)") opencv_core.Point point, int i3);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createRowSumFilter(int i, int i2, int i3);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createRowSumFilter(int i, int i2, int i3, int i4, int i5, @ByVal(nullValue = "cv::Scalar::all(0)") opencv_core.Scalar scalar);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createScharrFilter(int i, int i2, int i3, int i4);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createScharrFilter(int i, int i2, int i3, int i4, double d, int i5, int i6);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createSeparableLinearFilter(int i, int i2, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createSeparableLinearFilter(int i, int i2, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal(nullValue = "cv::Point(-1,-1)") opencv_core.Point point, int i3, int i4);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createSeparableLinearFilter(int i, int i2, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createSeparableLinearFilter(int i, int i2, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal(nullValue = "cv::Point(-1,-1)") opencv_core.Point point, int i3, int i4);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createSeparableLinearFilter(int i, int i2, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createSeparableLinearFilter(int i, int i2, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal(nullValue = "cv::Point(-1,-1)") opencv_core.Point point, int i3, int i4);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createSobelFilter(int i, int i2, int i3, int i4);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Filter createSobelFilter(int i, int i2, int i3, int i4, int i5, double d, int i6, int i7);

    static {
        Loader.load();
    }

    @Namespace("cv::cuda")
    public static class Filter extends opencv_core.Algorithm {
        public native void apply(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void apply(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void apply(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void apply(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void apply(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void apply(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        static {
            Loader.load();
        }

        public Filter(Pointer p) {
            super(p);
        }
    }
}
