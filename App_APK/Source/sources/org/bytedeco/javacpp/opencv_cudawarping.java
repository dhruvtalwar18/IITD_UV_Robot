package org.bytedeco.javacpp;

import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.opencv_core;

public class opencv_cudawarping extends org.bytedeco.javacpp.presets.opencv_cudawarping {
    @Namespace("cv::cuda")
    public static native void buildWarpAffineMaps(@ByVal opencv_core.GpuMat gpuMat, @Cast({"bool"}) boolean z, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv::cuda")
    public static native void buildWarpAffineMaps(@ByVal opencv_core.GpuMat gpuMat, @Cast({"bool"}) boolean z, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void buildWarpAffineMaps(@ByVal opencv_core.Mat mat, @Cast({"bool"}) boolean z, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv::cuda")
    public static native void buildWarpAffineMaps(@ByVal opencv_core.Mat mat, @Cast({"bool"}) boolean z, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void buildWarpAffineMaps(@ByVal opencv_core.UMat uMat, @Cast({"bool"}) boolean z, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv::cuda")
    public static native void buildWarpAffineMaps(@ByVal opencv_core.UMat uMat, @Cast({"bool"}) boolean z, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void buildWarpPerspectiveMaps(@ByVal opencv_core.GpuMat gpuMat, @Cast({"bool"}) boolean z, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv::cuda")
    public static native void buildWarpPerspectiveMaps(@ByVal opencv_core.GpuMat gpuMat, @Cast({"bool"}) boolean z, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void buildWarpPerspectiveMaps(@ByVal opencv_core.Mat mat, @Cast({"bool"}) boolean z, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv::cuda")
    public static native void buildWarpPerspectiveMaps(@ByVal opencv_core.Mat mat, @Cast({"bool"}) boolean z, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void buildWarpPerspectiveMaps(@ByVal opencv_core.UMat uMat, @Cast({"bool"}) boolean z, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv::cuda")
    public static native void buildWarpPerspectiveMaps(@ByVal opencv_core.UMat uMat, @Cast({"bool"}) boolean z, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void pyrDown(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void pyrDown(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void pyrDown(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void pyrDown(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void pyrDown(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void pyrDown(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void pyrUp(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void pyrUp(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void pyrUp(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void pyrUp(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void pyrUp(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void pyrUp(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void remap(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, int i);

    @Namespace("cv::cuda")
    public static native void remap(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, int i, int i2, @ByVal(nullValue = "cv::Scalar()") opencv_core.Scalar scalar, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void remap(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, int i);

    @Namespace("cv::cuda")
    public static native void remap(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, int i, int i2, @ByVal(nullValue = "cv::Scalar()") opencv_core.Scalar scalar, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void remap(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, int i);

    @Namespace("cv::cuda")
    public static native void remap(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, int i, int i2, @ByVal(nullValue = "cv::Scalar()") opencv_core.Scalar scalar, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void resize(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.Size size);

    @Namespace("cv::cuda")
    public static native void resize(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.Size size, double d, double d2, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void resize(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Size size);

    @Namespace("cv::cuda")
    public static native void resize(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Size size, double d, double d2, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void resize(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.Size size);

    @Namespace("cv::cuda")
    public static native void resize(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.Size size, double d, double d2, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void rotate(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.Size size, double d);

    @Namespace("cv::cuda")
    public static native void rotate(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.Size size, double d, double d2, double d3, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void rotate(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Size size, double d);

    @Namespace("cv::cuda")
    public static native void rotate(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Size size, double d, double d2, double d3, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void rotate(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.Size size, double d);

    @Namespace("cv::cuda")
    public static native void rotate(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.Size size, double d, double d2, double d3, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void warpAffine(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.Size size);

    @Namespace("cv::cuda")
    public static native void warpAffine(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.Size size, int i, int i2, @ByVal(nullValue = "cv::Scalar()") opencv_core.Scalar scalar, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void warpAffine(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Size size);

    @Namespace("cv::cuda")
    public static native void warpAffine(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Size size, int i, int i2, @ByVal(nullValue = "cv::Scalar()") opencv_core.Scalar scalar, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void warpAffine(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.Size size);

    @Namespace("cv::cuda")
    public static native void warpAffine(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.Size size, int i, int i2, @ByVal(nullValue = "cv::Scalar()") opencv_core.Scalar scalar, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void warpPerspective(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.Size size);

    @Namespace("cv::cuda")
    public static native void warpPerspective(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.Size size, int i, int i2, @ByVal(nullValue = "cv::Scalar()") opencv_core.Scalar scalar, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void warpPerspective(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Size size);

    @Namespace("cv::cuda")
    public static native void warpPerspective(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Size size, int i, int i2, @ByVal(nullValue = "cv::Scalar()") opencv_core.Scalar scalar, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void warpPerspective(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.Size size);

    @Namespace("cv::cuda")
    public static native void warpPerspective(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.Size size, int i, int i2, @ByVal(nullValue = "cv::Scalar()") opencv_core.Scalar scalar, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    static {
        Loader.load();
    }
}
