package org.bytedeco.javacpp;

import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.opencv_video;
import org.bytedeco.javacpp.presets.opencv_core;

public class opencv_optflow extends org.bytedeco.javacpp.presets.opencv_optflow {
    @Namespace("cv::motempl")
    public static native double calcGlobalOrientation(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, double d, double d2);

    @Namespace("cv::motempl")
    public static native double calcGlobalOrientation(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, double d, double d2);

    @Namespace("cv::motempl")
    public static native double calcGlobalOrientation(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, double d, double d2);

    @Namespace("cv::motempl")
    public static native void calcMotionGradient(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, double d, double d2);

    @Namespace("cv::motempl")
    public static native void calcMotionGradient(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, double d, double d2, int i);

    @Namespace("cv::motempl")
    public static native void calcMotionGradient(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, double d, double d2);

    @Namespace("cv::motempl")
    public static native void calcMotionGradient(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, double d, double d2, int i);

    @Namespace("cv::motempl")
    public static native void calcMotionGradient(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, double d, double d2);

    @Namespace("cv::motempl")
    public static native void calcMotionGradient(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, double d, double d2, int i);

    @Namespace("cv::optflow")
    public static native void calcOpticalFlowSF(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, int i, int i2, int i3);

    @Namespace("cv::optflow")
    public static native void calcOpticalFlowSF(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, int i, int i2, int i3, double d, double d2, int i4, double d3, double d4, double d5, int i5, double d6, double d7, double d8);

    @Namespace("cv::optflow")
    public static native void calcOpticalFlowSF(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, int i, int i2, int i3);

    @Namespace("cv::optflow")
    public static native void calcOpticalFlowSF(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, int i, int i2, int i3, double d, double d2, int i4, double d3, double d4, double d5, int i5, double d6, double d7, double d8);

    @Namespace("cv::optflow")
    public static native void calcOpticalFlowSF(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, int i, int i2, int i3);

    @Namespace("cv::optflow")
    public static native void calcOpticalFlowSF(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, int i, int i2, int i3, double d, double d2, int i4, double d3, double d4, double d5, int i5, double d6, double d7, double d8);

    @Namespace("cv::optflow")
    public static native void calcOpticalFlowSparseToDense(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv::optflow")
    public static native void calcOpticalFlowSparseToDense(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, int i, int i2, float f, @Cast({"bool"}) boolean z, float f2, float f3);

    @Namespace("cv::optflow")
    public static native void calcOpticalFlowSparseToDense(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv::optflow")
    public static native void calcOpticalFlowSparseToDense(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, int i, int i2, float f, @Cast({"bool"}) boolean z, float f2, float f3);

    @Namespace("cv::optflow")
    public static native void calcOpticalFlowSparseToDense(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv::optflow")
    public static native void calcOpticalFlowSparseToDense(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, int i, int i2, float f, @Cast({"bool"}) boolean z, float f2, float f3);

    @Namespace("cv::optflow")
    @opencv_core.Ptr
    public static native opencv_video.DenseOpticalFlow createOptFlow_DeepFlow();

    @Namespace("cv::optflow")
    @opencv_core.Ptr
    public static native DualTVL1OpticalFlow createOptFlow_DualTVL1();

    @Namespace("cv::optflow")
    @opencv_core.Ptr
    public static native opencv_video.DenseOpticalFlow createOptFlow_Farneback();

    @Namespace("cv::optflow")
    @opencv_core.Ptr
    public static native opencv_video.DenseOpticalFlow createOptFlow_SimpleFlow();

    @Namespace("cv::optflow")
    @opencv_core.Ptr
    public static native opencv_video.DenseOpticalFlow createOptFlow_SparseToDense();

    @Namespace("cv::motempl")
    public static native void segmentMotion(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef opencv_core.RectVector rectVector, double d, double d2);

    @Namespace("cv::motempl")
    public static native void segmentMotion(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef opencv_core.RectVector rectVector, double d, double d2);

    @Namespace("cv::motempl")
    public static native void segmentMotion(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef opencv_core.RectVector rectVector, double d, double d2);

    @Namespace("cv::motempl")
    public static native void updateMotionHistory(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, double d, double d2);

    @Namespace("cv::motempl")
    public static native void updateMotionHistory(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, double d, double d2);

    @Namespace("cv::motempl")
    public static native void updateMotionHistory(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, double d, double d2);

    static {
        Loader.load();
    }

    @Namespace("cv::optflow")
    public static class DualTVL1OpticalFlow extends opencv_video.DenseOpticalFlow {
        @opencv_core.Ptr
        public static native DualTVL1OpticalFlow create();

        @opencv_core.Ptr
        public static native DualTVL1OpticalFlow create(double d, double d2, double d3, int i, int i2, double d4, int i3, int i4, double d5, double d6, int i5, @Cast({"bool"}) boolean z);

        public native double getEpsilon();

        public native double getGamma();

        public native int getInnerIterations();

        public native double getLambda();

        public native int getMedianFiltering();

        public native int getOuterIterations();

        public native double getScaleStep();

        public native int getScalesNumber();

        public native double getTau();

        public native double getTheta();

        @Cast({"bool"})
        public native boolean getUseInitialFlow();

        public native int getWarpingsNumber();

        public native void setEpsilon(double d);

        public native void setGamma(double d);

        public native void setInnerIterations(int i);

        public native void setLambda(double d);

        public native void setMedianFiltering(int i);

        public native void setOuterIterations(int i);

        public native void setScaleStep(double d);

        public native void setScalesNumber(int i);

        public native void setTau(double d);

        public native void setTheta(double d);

        public native void setUseInitialFlow(@Cast({"bool"}) boolean z);

        public native void setWarpingsNumber(int i);

        static {
            Loader.load();
        }

        public DualTVL1OpticalFlow(Pointer p) {
            super(p);
        }
    }
}
