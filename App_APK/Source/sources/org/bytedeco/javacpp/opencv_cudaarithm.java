package org.bytedeco.javacpp;

import java.nio.DoubleBuffer;
import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Const;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.presets.opencv_core;

public class opencv_cudaarithm extends org.bytedeco.javacpp.presets.opencv_cudaarithm {
    @Namespace("cv::cuda")
    public static native void abs(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void abs(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void abs(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void abs(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void abs(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void abs(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    @ByVal
    public static native opencv_core.Scalar absSum(@ByVal opencv_core.GpuMat gpuMat);

    @Namespace("cv::cuda")
    @ByVal
    public static native opencv_core.Scalar absSum(@ByVal opencv_core.GpuMat gpuMat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    @ByVal
    public static native opencv_core.Scalar absSum(@ByVal opencv_core.Mat mat);

    @Namespace("cv::cuda")
    @ByVal
    public static native opencv_core.Scalar absSum(@ByVal opencv_core.Mat mat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    @ByVal
    public static native opencv_core.Scalar absSum(@ByVal opencv_core.UMat uMat);

    @Namespace("cv::cuda")
    @ByVal
    public static native opencv_core.Scalar absSum(@ByVal opencv_core.UMat uMat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void absdiff(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv::cuda")
    public static native void absdiff(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void absdiff(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv::cuda")
    public static native void absdiff(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void absdiff(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv::cuda")
    public static native void absdiff(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void add(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv::cuda")
    public static native void add(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat4, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void add(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv::cuda")
    public static native void add(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat4, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void add(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv::cuda")
    public static native void add(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat4, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void addWeighted(@ByVal opencv_core.GpuMat gpuMat, double d, @ByVal opencv_core.GpuMat gpuMat2, double d2, double d3, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv::cuda")
    public static native void addWeighted(@ByVal opencv_core.GpuMat gpuMat, double d, @ByVal opencv_core.GpuMat gpuMat2, double d2, double d3, @ByVal opencv_core.GpuMat gpuMat3, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void addWeighted(@ByVal opencv_core.Mat mat, double d, @ByVal opencv_core.Mat mat2, double d2, double d3, @ByVal opencv_core.Mat mat3);

    @Namespace("cv::cuda")
    public static native void addWeighted(@ByVal opencv_core.Mat mat, double d, @ByVal opencv_core.Mat mat2, double d2, double d3, @ByVal opencv_core.Mat mat3, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void addWeighted(@ByVal opencv_core.UMat uMat, double d, @ByVal opencv_core.UMat uMat2, double d2, double d3, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv::cuda")
    public static native void addWeighted(@ByVal opencv_core.UMat uMat, double d, @ByVal opencv_core.UMat uMat2, double d2, double d3, @ByVal opencv_core.UMat uMat3, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void bitwise_and(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv::cuda")
    public static native void bitwise_and(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat4, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void bitwise_and(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv::cuda")
    public static native void bitwise_and(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat4, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void bitwise_and(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv::cuda")
    public static native void bitwise_and(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat4, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void bitwise_not(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void bitwise_not(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void bitwise_not(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void bitwise_not(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void bitwise_not(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void bitwise_not(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void bitwise_or(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv::cuda")
    public static native void bitwise_or(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat4, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void bitwise_or(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv::cuda")
    public static native void bitwise_or(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat4, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void bitwise_or(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv::cuda")
    public static native void bitwise_or(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat4, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void bitwise_xor(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv::cuda")
    public static native void bitwise_xor(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat4, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void bitwise_xor(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv::cuda")
    public static native void bitwise_xor(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat4, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void bitwise_xor(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv::cuda")
    public static native void bitwise_xor(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat4, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void calcAbsSum(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void calcAbsSum(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void calcAbsSum(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void calcAbsSum(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void calcAbsSum(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void calcAbsSum(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void calcNorm(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i);

    @Namespace("cv::cuda")
    public static native void calcNorm(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void calcNorm(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i);

    @Namespace("cv::cuda")
    public static native void calcNorm(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void calcNorm(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i);

    @Namespace("cv::cuda")
    public static native void calcNorm(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void calcNormDiff(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv::cuda")
    public static native void calcNormDiff(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void calcNormDiff(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv::cuda")
    public static native void calcNormDiff(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void calcNormDiff(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv::cuda")
    public static native void calcNormDiff(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void calcSqrSum(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void calcSqrSum(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void calcSqrSum(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void calcSqrSum(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void calcSqrSum(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void calcSqrSum(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void calcSum(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void calcSum(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void calcSum(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void calcSum(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void calcSum(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void calcSum(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void cartToPolar(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4);

    @Namespace("cv::cuda")
    public static native void cartToPolar(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @Cast({"bool"}) boolean z, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void cartToPolar(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4);

    @Namespace("cv::cuda")
    public static native void cartToPolar(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @Cast({"bool"}) boolean z, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void cartToPolar(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4);

    @Namespace("cv::cuda")
    public static native void cartToPolar(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @Cast({"bool"}) boolean z, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void compare(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, int i);

    @Namespace("cv::cuda")
    public static native void compare(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void compare(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, int i);

    @Namespace("cv::cuda")
    public static native void compare(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void compare(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, int i);

    @Namespace("cv::cuda")
    public static native void compare(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void copyMakeBorder(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, int i2, int i3, int i4, int i5);

    @Namespace("cv::cuda")
    public static native void copyMakeBorder(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, int i2, int i3, int i4, int i5, @ByVal(nullValue = "cv::Scalar()") opencv_core.Scalar scalar, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void copyMakeBorder(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, int i2, int i3, int i4, int i5);

    @Namespace("cv::cuda")
    public static native void copyMakeBorder(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, int i2, int i3, int i4, int i5, @ByVal(nullValue = "cv::Scalar()") opencv_core.Scalar scalar, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void copyMakeBorder(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, int i2, int i3, int i4, int i5);

    @Namespace("cv::cuda")
    public static native void copyMakeBorder(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, int i2, int i3, int i4, int i5, @ByVal(nullValue = "cv::Scalar()") opencv_core.Scalar scalar, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native int countNonZero(@ByVal opencv_core.GpuMat gpuMat);

    @Namespace("cv::cuda")
    public static native int countNonZero(@ByVal opencv_core.Mat mat);

    @Namespace("cv::cuda")
    public static native int countNonZero(@ByVal opencv_core.UMat uMat);

    @Namespace("cv::cuda")
    public static native void countNonZero(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void countNonZero(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void countNonZero(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void countNonZero(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void countNonZero(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void countNonZero(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Convolution createConvolution();

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native Convolution createConvolution(@ByVal(nullValue = "cv::Size()") opencv_core.Size size);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native DFT createDFT(@ByVal opencv_core.Size size, int i);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native LookUpTable createLookUpTable(@ByVal opencv_core.GpuMat gpuMat);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native LookUpTable createLookUpTable(@ByVal opencv_core.Mat mat);

    @Namespace("cv::cuda")
    @opencv_core.Ptr
    public static native LookUpTable createLookUpTable(@ByVal opencv_core.UMat uMat);

    @Namespace("cv::cuda")
    public static native void dft(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.Size size);

    @Namespace("cv::cuda")
    public static native void dft(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.Size size, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void dft(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Size size);

    @Namespace("cv::cuda")
    public static native void dft(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Size size, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void dft(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.Size size);

    @Namespace("cv::cuda")
    public static native void dft(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.Size size, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void divide(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv::cuda")
    public static native void divide(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, double d, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void divide(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv::cuda")
    public static native void divide(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, double d, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void divide(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv::cuda")
    public static native void divide(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, double d, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void exp(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void exp(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void exp(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void exp(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void exp(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void exp(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void findMinMax(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void findMinMax(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void findMinMax(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void findMinMax(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void findMinMax(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void findMinMax(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void findMinMaxLoc(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv::cuda")
    public static native void findMinMaxLoc(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat4, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void findMinMaxLoc(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv::cuda")
    public static native void findMinMaxLoc(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat4, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void findMinMaxLoc(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv::cuda")
    public static native void findMinMaxLoc(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat4, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void flip(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i);

    @Namespace("cv::cuda")
    public static native void flip(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void flip(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i);

    @Namespace("cv::cuda")
    public static native void flip(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void flip(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i);

    @Namespace("cv::cuda")
    public static native void flip(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void gemm(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, double d, @ByVal opencv_core.GpuMat gpuMat3, double d2, @ByVal opencv_core.GpuMat gpuMat4);

    @Namespace("cv::cuda")
    public static native void gemm(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, double d, @ByVal opencv_core.GpuMat gpuMat3, double d2, @ByVal opencv_core.GpuMat gpuMat4, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void gemm(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, double d, @ByVal opencv_core.Mat mat3, double d2, @ByVal opencv_core.Mat mat4);

    @Namespace("cv::cuda")
    public static native void gemm(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, double d, @ByVal opencv_core.Mat mat3, double d2, @ByVal opencv_core.Mat mat4, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void gemm(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, double d, @ByVal opencv_core.UMat uMat3, double d2, @ByVal opencv_core.UMat uMat4);

    @Namespace("cv::cuda")
    public static native void gemm(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, double d, @ByVal opencv_core.UMat uMat3, double d2, @ByVal opencv_core.UMat uMat4, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void integral(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void integral(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void integral(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void integral(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void integral(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void integral(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void log(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void log(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void log(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void log(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void log(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void log(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void lshift(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.Scalar4i scalar4i, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void lshift(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.Scalar4i scalar4i, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void lshift(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Scalar4i scalar4i, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void lshift(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Scalar4i scalar4i, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void lshift(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.Scalar4i scalar4i, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void lshift(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.Scalar4i scalar4i, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void magnitude(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void magnitude(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv::cuda")
    public static native void magnitude(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void magnitude(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void magnitude(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void magnitude(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv::cuda")
    public static native void magnitude(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void magnitude(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void magnitude(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void magnitude(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void magnitude(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv::cuda")
    public static native void magnitude(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void magnitudeSqr(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void magnitudeSqr(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv::cuda")
    public static native void magnitudeSqr(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void magnitudeSqr(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void magnitudeSqr(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void magnitudeSqr(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv::cuda")
    public static native void magnitudeSqr(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void magnitudeSqr(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void magnitudeSqr(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void magnitudeSqr(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void magnitudeSqr(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv::cuda")
    public static native void magnitudeSqr(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void max(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv::cuda")
    public static native void max(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void max(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv::cuda")
    public static native void max(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void max(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv::cuda")
    public static native void max(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void meanStdDev(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void meanStdDev(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void meanStdDev(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.Scalar scalar, @ByRef opencv_core.Scalar scalar2);

    @Namespace("cv::cuda")
    public static native void meanStdDev(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void meanStdDev(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void meanStdDev(@ByVal opencv_core.Mat mat, @ByRef opencv_core.Scalar scalar, @ByRef opencv_core.Scalar scalar2);

    @Namespace("cv::cuda")
    public static native void meanStdDev(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.Scalar scalar, @ByRef opencv_core.Scalar scalar2);

    @Namespace("cv::cuda")
    public static native void meanStdDev(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void meanStdDev(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void merge(@Const opencv_core.GpuMat gpuMat, @Cast({"size_t"}) long j, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void merge(@Const opencv_core.GpuMat gpuMat, @Cast({"size_t"}) long j, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void merge(@Const opencv_core.GpuMat gpuMat, @Cast({"size_t"}) long j, @ByVal opencv_core.Mat mat);

    @Namespace("cv::cuda")
    public static native void merge(@Const opencv_core.GpuMat gpuMat, @Cast({"size_t"}) long j, @ByVal opencv_core.Mat mat, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void merge(@Const opencv_core.GpuMat gpuMat, @Cast({"size_t"}) long j, @ByVal opencv_core.UMat uMat);

    @Namespace("cv::cuda")
    public static native void merge(@Const opencv_core.GpuMat gpuMat, @Cast({"size_t"}) long j, @ByVal opencv_core.UMat uMat, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void merge(@ByRef @Const opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat);

    @Namespace("cv::cuda")
    public static native void merge(@ByRef @Const opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void merge(@ByRef @Const opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat);

    @Namespace("cv::cuda")
    public static native void merge(@ByRef @Const opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void merge(@ByRef @Const opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat);

    @Namespace("cv::cuda")
    public static native void merge(@ByRef @Const opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void min(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv::cuda")
    public static native void min(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void min(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv::cuda")
    public static native void min(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void min(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv::cuda")
    public static native void min(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void minMax(@ByVal opencv_core.GpuMat gpuMat, DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2);

    @Namespace("cv::cuda")
    public static native void minMax(@ByVal opencv_core.GpuMat gpuMat, DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void minMax(@ByVal opencv_core.GpuMat gpuMat, DoublePointer doublePointer, DoublePointer doublePointer2);

    @Namespace("cv::cuda")
    public static native void minMax(@ByVal opencv_core.GpuMat gpuMat, DoublePointer doublePointer, DoublePointer doublePointer2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void minMax(@ByVal opencv_core.GpuMat gpuMat, double[] dArr, double[] dArr2);

    @Namespace("cv::cuda")
    public static native void minMax(@ByVal opencv_core.GpuMat gpuMat, double[] dArr, double[] dArr2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void minMax(@ByVal opencv_core.Mat mat, DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2);

    @Namespace("cv::cuda")
    public static native void minMax(@ByVal opencv_core.Mat mat, DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void minMax(@ByVal opencv_core.Mat mat, DoublePointer doublePointer, DoublePointer doublePointer2);

    @Namespace("cv::cuda")
    public static native void minMax(@ByVal opencv_core.Mat mat, DoublePointer doublePointer, DoublePointer doublePointer2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void minMax(@ByVal opencv_core.Mat mat, double[] dArr, double[] dArr2);

    @Namespace("cv::cuda")
    public static native void minMax(@ByVal opencv_core.Mat mat, double[] dArr, double[] dArr2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void minMax(@ByVal opencv_core.UMat uMat, DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2);

    @Namespace("cv::cuda")
    public static native void minMax(@ByVal opencv_core.UMat uMat, DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void minMax(@ByVal opencv_core.UMat uMat, DoublePointer doublePointer, DoublePointer doublePointer2);

    @Namespace("cv::cuda")
    public static native void minMax(@ByVal opencv_core.UMat uMat, DoublePointer doublePointer, DoublePointer doublePointer2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void minMax(@ByVal opencv_core.UMat uMat, double[] dArr, double[] dArr2);

    @Namespace("cv::cuda")
    public static native void minMax(@ByVal opencv_core.UMat uMat, double[] dArr, double[] dArr2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void minMaxLoc(@ByVal opencv_core.GpuMat gpuMat, DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2, opencv_core.Point point, opencv_core.Point point2);

    @Namespace("cv::cuda")
    public static native void minMaxLoc(@ByVal opencv_core.GpuMat gpuMat, DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2, opencv_core.Point point, opencv_core.Point point2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void minMaxLoc(@ByVal opencv_core.GpuMat gpuMat, DoublePointer doublePointer, DoublePointer doublePointer2, opencv_core.Point point, opencv_core.Point point2);

    @Namespace("cv::cuda")
    public static native void minMaxLoc(@ByVal opencv_core.GpuMat gpuMat, DoublePointer doublePointer, DoublePointer doublePointer2, opencv_core.Point point, opencv_core.Point point2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void minMaxLoc(@ByVal opencv_core.GpuMat gpuMat, double[] dArr, double[] dArr2, opencv_core.Point point, opencv_core.Point point2);

    @Namespace("cv::cuda")
    public static native void minMaxLoc(@ByVal opencv_core.GpuMat gpuMat, double[] dArr, double[] dArr2, opencv_core.Point point, opencv_core.Point point2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void minMaxLoc(@ByVal opencv_core.Mat mat, DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2, opencv_core.Point point, opencv_core.Point point2);

    @Namespace("cv::cuda")
    public static native void minMaxLoc(@ByVal opencv_core.Mat mat, DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2, opencv_core.Point point, opencv_core.Point point2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void minMaxLoc(@ByVal opencv_core.Mat mat, DoublePointer doublePointer, DoublePointer doublePointer2, opencv_core.Point point, opencv_core.Point point2);

    @Namespace("cv::cuda")
    public static native void minMaxLoc(@ByVal opencv_core.Mat mat, DoublePointer doublePointer, DoublePointer doublePointer2, opencv_core.Point point, opencv_core.Point point2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void minMaxLoc(@ByVal opencv_core.Mat mat, double[] dArr, double[] dArr2, opencv_core.Point point, opencv_core.Point point2);

    @Namespace("cv::cuda")
    public static native void minMaxLoc(@ByVal opencv_core.Mat mat, double[] dArr, double[] dArr2, opencv_core.Point point, opencv_core.Point point2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void minMaxLoc(@ByVal opencv_core.UMat uMat, DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2, opencv_core.Point point, opencv_core.Point point2);

    @Namespace("cv::cuda")
    public static native void minMaxLoc(@ByVal opencv_core.UMat uMat, DoubleBuffer doubleBuffer, DoubleBuffer doubleBuffer2, opencv_core.Point point, opencv_core.Point point2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void minMaxLoc(@ByVal opencv_core.UMat uMat, DoublePointer doublePointer, DoublePointer doublePointer2, opencv_core.Point point, opencv_core.Point point2);

    @Namespace("cv::cuda")
    public static native void minMaxLoc(@ByVal opencv_core.UMat uMat, DoublePointer doublePointer, DoublePointer doublePointer2, opencv_core.Point point, opencv_core.Point point2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void minMaxLoc(@ByVal opencv_core.UMat uMat, double[] dArr, double[] dArr2, opencv_core.Point point, opencv_core.Point point2);

    @Namespace("cv::cuda")
    public static native void minMaxLoc(@ByVal opencv_core.UMat uMat, double[] dArr, double[] dArr2, opencv_core.Point point, opencv_core.Point point2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void mulAndScaleSpectrums(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, int i, float f);

    @Namespace("cv::cuda")
    public static native void mulAndScaleSpectrums(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, int i, float f, @Cast({"bool"}) boolean z, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void mulAndScaleSpectrums(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, int i, float f);

    @Namespace("cv::cuda")
    public static native void mulAndScaleSpectrums(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, int i, float f, @Cast({"bool"}) boolean z, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void mulAndScaleSpectrums(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, int i, float f);

    @Namespace("cv::cuda")
    public static native void mulAndScaleSpectrums(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, int i, float f, @Cast({"bool"}) boolean z, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void mulSpectrums(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, int i);

    @Namespace("cv::cuda")
    public static native void mulSpectrums(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, int i, @Cast({"bool"}) boolean z, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void mulSpectrums(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, int i);

    @Namespace("cv::cuda")
    public static native void mulSpectrums(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, int i, @Cast({"bool"}) boolean z, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void mulSpectrums(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, int i);

    @Namespace("cv::cuda")
    public static native void mulSpectrums(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, int i, @Cast({"bool"}) boolean z, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void multiply(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv::cuda")
    public static native void multiply(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, double d, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void multiply(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv::cuda")
    public static native void multiply(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, double d, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void multiply(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv::cuda")
    public static native void multiply(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, double d, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native double norm(@ByVal opencv_core.GpuMat gpuMat, int i);

    @Namespace("cv::cuda")
    public static native double norm(@ByVal opencv_core.GpuMat gpuMat, int i, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native double norm(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native double norm(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i);

    @Namespace("cv::cuda")
    public static native double norm(@ByVal opencv_core.Mat mat, int i);

    @Namespace("cv::cuda")
    public static native double norm(@ByVal opencv_core.Mat mat, int i, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native double norm(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native double norm(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i);

    @Namespace("cv::cuda")
    public static native double norm(@ByVal opencv_core.UMat uMat, int i);

    @Namespace("cv::cuda")
    public static native double norm(@ByVal opencv_core.UMat uMat, int i, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native double norm(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native double norm(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i);

    @Namespace("cv::cuda")
    public static native void normalize(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, double d, double d2, int i, int i2);

    @Namespace("cv::cuda")
    public static native void normalize(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, double d, double d2, int i, int i2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void normalize(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, double d, double d2, int i, int i2);

    @Namespace("cv::cuda")
    public static native void normalize(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, double d, double d2, int i, int i2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void normalize(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, double d, double d2, int i, int i2);

    @Namespace("cv::cuda")
    public static native void normalize(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, double d, double d2, int i, int i2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void phase(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv::cuda")
    public static native void phase(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @Cast({"bool"}) boolean z, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void phase(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv::cuda")
    public static native void phase(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @Cast({"bool"}) boolean z, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void phase(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv::cuda")
    public static native void phase(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @Cast({"bool"}) boolean z, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void polarToCart(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4);

    @Namespace("cv::cuda")
    public static native void polarToCart(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @Cast({"bool"}) boolean z, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void polarToCart(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4);

    @Namespace("cv::cuda")
    public static native void polarToCart(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @Cast({"bool"}) boolean z, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void polarToCart(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4);

    @Namespace("cv::cuda")
    public static native void polarToCart(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @Cast({"bool"}) boolean z, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void pow(@ByVal opencv_core.GpuMat gpuMat, double d, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void pow(@ByVal opencv_core.GpuMat gpuMat, double d, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void pow(@ByVal opencv_core.Mat mat, double d, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void pow(@ByVal opencv_core.Mat mat, double d, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void pow(@ByVal opencv_core.UMat uMat, double d, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void pow(@ByVal opencv_core.UMat uMat, double d, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void rectStdDev(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.Rect rect);

    @Namespace("cv::cuda")
    public static native void rectStdDev(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.Rect rect, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void rectStdDev(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Rect rect);

    @Namespace("cv::cuda")
    public static native void rectStdDev(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Rect rect, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void rectStdDev(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.Rect rect);

    @Namespace("cv::cuda")
    public static native void rectStdDev(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.Rect rect, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void reduce(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, int i2);

    @Namespace("cv::cuda")
    public static native void reduce(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, int i2, int i3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void reduce(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, int i2);

    @Namespace("cv::cuda")
    public static native void reduce(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, int i2, int i3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void reduce(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, int i2);

    @Namespace("cv::cuda")
    public static native void reduce(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, int i2, int i3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void rshift(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.Scalar4i scalar4i, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void rshift(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.Scalar4i scalar4i, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void rshift(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Scalar4i scalar4i, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void rshift(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Scalar4i scalar4i, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void rshift(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.Scalar4i scalar4i, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void rshift(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.Scalar4i scalar4i, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void scaleAdd(@ByVal opencv_core.GpuMat gpuMat, double d, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv::cuda")
    public static native void scaleAdd(@ByVal opencv_core.GpuMat gpuMat, double d, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void scaleAdd(@ByVal opencv_core.Mat mat, double d, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv::cuda")
    public static native void scaleAdd(@ByVal opencv_core.Mat mat, double d, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void scaleAdd(@ByVal opencv_core.UMat uMat, double d, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv::cuda")
    public static native void scaleAdd(@ByVal opencv_core.UMat uMat, double d, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void split(@ByVal opencv_core.GpuMat gpuMat, opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void split(@ByVal opencv_core.GpuMat gpuMat, opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void split(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.GpuMatVector gpuMatVector);

    @Namespace("cv::cuda")
    public static native void split(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.GpuMatVector gpuMatVector, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void split(@ByVal opencv_core.Mat mat, opencv_core.GpuMat gpuMat);

    @Namespace("cv::cuda")
    public static native void split(@ByVal opencv_core.Mat mat, opencv_core.GpuMat gpuMat, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void split(@ByVal opencv_core.Mat mat, @ByRef opencv_core.GpuMatVector gpuMatVector);

    @Namespace("cv::cuda")
    public static native void split(@ByVal opencv_core.Mat mat, @ByRef opencv_core.GpuMatVector gpuMatVector, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void split(@ByVal opencv_core.UMat uMat, opencv_core.GpuMat gpuMat);

    @Namespace("cv::cuda")
    public static native void split(@ByVal opencv_core.UMat uMat, opencv_core.GpuMat gpuMat, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void split(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.GpuMatVector gpuMatVector);

    @Namespace("cv::cuda")
    public static native void split(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.GpuMatVector gpuMatVector, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void sqr(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void sqr(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void sqr(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void sqr(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void sqr(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void sqr(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void sqrIntegral(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void sqrIntegral(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void sqrIntegral(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void sqrIntegral(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void sqrIntegral(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void sqrIntegral(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    @ByVal
    public static native opencv_core.Scalar sqrSum(@ByVal opencv_core.GpuMat gpuMat);

    @Namespace("cv::cuda")
    @ByVal
    public static native opencv_core.Scalar sqrSum(@ByVal opencv_core.GpuMat gpuMat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    @ByVal
    public static native opencv_core.Scalar sqrSum(@ByVal opencv_core.Mat mat);

    @Namespace("cv::cuda")
    @ByVal
    public static native opencv_core.Scalar sqrSum(@ByVal opencv_core.Mat mat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    @ByVal
    public static native opencv_core.Scalar sqrSum(@ByVal opencv_core.UMat uMat);

    @Namespace("cv::cuda")
    @ByVal
    public static native opencv_core.Scalar sqrSum(@ByVal opencv_core.UMat uMat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void sqrt(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void sqrt(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void sqrt(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void sqrt(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void sqrt(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void sqrt(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void subtract(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv::cuda")
    public static native void subtract(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat4, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void subtract(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv::cuda")
    public static native void subtract(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat4, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void subtract(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv::cuda")
    public static native void subtract(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat4, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    @ByVal
    public static native opencv_core.Scalar sum(@ByVal opencv_core.GpuMat gpuMat);

    @Namespace("cv::cuda")
    @ByVal
    public static native opencv_core.Scalar sum(@ByVal opencv_core.GpuMat gpuMat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    @ByVal
    public static native opencv_core.Scalar sum(@ByVal opencv_core.Mat mat);

    @Namespace("cv::cuda")
    @ByVal
    public static native opencv_core.Scalar sum(@ByVal opencv_core.Mat mat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    @ByVal
    public static native opencv_core.Scalar sum(@ByVal opencv_core.UMat uMat);

    @Namespace("cv::cuda")
    @ByVal
    public static native opencv_core.Scalar sum(@ByVal opencv_core.UMat uMat, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native double threshold(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, double d, double d2, int i);

    @Namespace("cv::cuda")
    public static native double threshold(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, double d, double d2, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native double threshold(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, double d, double d2, int i);

    @Namespace("cv::cuda")
    public static native double threshold(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, double d, double d2, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native double threshold(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, double d, double d2, int i);

    @Namespace("cv::cuda")
    public static native double threshold(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, double d, double d2, int i, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void transpose(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::cuda")
    public static native void transpose(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void transpose(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::cuda")
    public static native void transpose(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    @Namespace("cv::cuda")
    public static native void transpose(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::cuda")
    public static native void transpose(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

    static {
        Loader.load();
    }

    @Namespace("cv::cuda")
    public static class LookUpTable extends opencv_core.Algorithm {
        public native void transform(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void transform(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void transform(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void transform(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void transform(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void transform(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        static {
            Loader.load();
        }

        public LookUpTable(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::cuda")
    public static class DFT extends opencv_core.Algorithm {
        public native void compute(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void compute(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void compute(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void compute(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void compute(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void compute(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        static {
            Loader.load();
        }

        public DFT(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::cuda")
    public static class Convolution extends opencv_core.Algorithm {
        public native void convolve(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

        public native void convolve(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @Cast({"bool"}) boolean z, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void convolve(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

        public native void convolve(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @Cast({"bool"}) boolean z, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void convolve(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

        public native void convolve(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @Cast({"bool"}) boolean z, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        static {
            Loader.load();
        }

        public Convolution(Pointer p) {
            super(p);
        }
    }
}
