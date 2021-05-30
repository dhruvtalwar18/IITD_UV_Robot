package org.bytedeco.javacpp;

import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.presets.opencv_core;

public class opencv_cudaoptflow extends org.bytedeco.javacpp.presets.opencv_cudaoptflow {
    static {
        Loader.load();
    }

    @Namespace("cv::cuda")
    public static class DenseOpticalFlow extends opencv_core.Algorithm {
        public native void calc(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

        public native void calc(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void calc(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

        public native void calc(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void calc(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

        public native void calc(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        static {
            Loader.load();
        }

        public DenseOpticalFlow(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::cuda")
    public static class SparseOpticalFlow extends opencv_core.Algorithm {
        public native void calc(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5);

        public native void calc(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat6, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void calc(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5);

        public native void calc(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat6, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void calc(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5);

        public native void calc(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat6, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        static {
            Loader.load();
        }

        public SparseOpticalFlow(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::cuda")
    public static class BroxOpticalFlow extends DenseOpticalFlow {
        @opencv_core.Ptr
        public static native BroxOpticalFlow create();

        @opencv_core.Ptr
        public static native BroxOpticalFlow create(double d, double d2, double d3, int i, int i2, int i3);

        public native double getFlowSmoothness();

        public native double getGradientConstancyImportance();

        public native int getInnerIterations();

        public native int getOuterIterations();

        public native double getPyramidScaleFactor();

        public native int getSolverIterations();

        public native void setFlowSmoothness(double d);

        public native void setGradientConstancyImportance(double d);

        public native void setInnerIterations(int i);

        public native void setOuterIterations(int i);

        public native void setPyramidScaleFactor(double d);

        public native void setSolverIterations(int i);

        static {
            Loader.load();
        }

        public BroxOpticalFlow(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::cuda")
    public static class SparsePyrLKOpticalFlow extends SparseOpticalFlow {
        @opencv_core.Ptr
        public static native SparsePyrLKOpticalFlow create();

        @opencv_core.Ptr
        public static native SparsePyrLKOpticalFlow create(@ByVal(nullValue = "cv::Size(21, 21)") opencv_core.Size size, int i, int i2, @Cast({"bool"}) boolean z);

        public native int getMaxLevel();

        public native int getNumIters();

        @Cast({"bool"})
        public native boolean getUseInitialFlow();

        @ByVal
        public native opencv_core.Size getWinSize();

        public native void setMaxLevel(int i);

        public native void setNumIters(int i);

        public native void setUseInitialFlow(@Cast({"bool"}) boolean z);

        public native void setWinSize(@ByVal opencv_core.Size size);

        static {
            Loader.load();
        }

        public SparsePyrLKOpticalFlow(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::cuda")
    public static class DensePyrLKOpticalFlow extends DenseOpticalFlow {
        @opencv_core.Ptr
        public static native DensePyrLKOpticalFlow create();

        @opencv_core.Ptr
        public static native DensePyrLKOpticalFlow create(@ByVal(nullValue = "cv::Size(13, 13)") opencv_core.Size size, int i, int i2, @Cast({"bool"}) boolean z);

        public native int getMaxLevel();

        public native int getNumIters();

        @Cast({"bool"})
        public native boolean getUseInitialFlow();

        @ByVal
        public native opencv_core.Size getWinSize();

        public native void setMaxLevel(int i);

        public native void setNumIters(int i);

        public native void setUseInitialFlow(@Cast({"bool"}) boolean z);

        public native void setWinSize(@ByVal opencv_core.Size size);

        static {
            Loader.load();
        }

        public DensePyrLKOpticalFlow(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::cuda")
    public static class FarnebackOpticalFlow extends DenseOpticalFlow {
        @opencv_core.Ptr
        public static native FarnebackOpticalFlow create();

        @opencv_core.Ptr
        public static native FarnebackOpticalFlow create(int i, double d, @Cast({"bool"}) boolean z, int i2, int i3, int i4, double d2, int i5);

        @Cast({"bool"})
        public native boolean getFastPyramids();

        public native int getFlags();

        public native int getNumIters();

        public native int getNumLevels();

        public native int getPolyN();

        public native double getPolySigma();

        public native double getPyrScale();

        public native int getWinSize();

        public native void setFastPyramids(@Cast({"bool"}) boolean z);

        public native void setFlags(int i);

        public native void setNumIters(int i);

        public native void setNumLevels(int i);

        public native void setPolyN(int i);

        public native void setPolySigma(double d);

        public native void setPyrScale(double d);

        public native void setWinSize(int i);

        static {
            Loader.load();
        }

        public FarnebackOpticalFlow(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::cuda")
    public static class OpticalFlowDual_TVL1 extends DenseOpticalFlow {
        @opencv_core.Ptr
        public static native OpticalFlowDual_TVL1 create();

        @opencv_core.Ptr
        public static native OpticalFlowDual_TVL1 create(double d, double d2, double d3, int i, int i2, double d4, int i3, double d5, double d6, @Cast({"bool"}) boolean z);

        public native double getEpsilon();

        public native double getGamma();

        public native double getLambda();

        public native int getNumIterations();

        public native int getNumScales();

        public native int getNumWarps();

        public native double getScaleStep();

        public native double getTau();

        public native double getTheta();

        @Cast({"bool"})
        public native boolean getUseInitialFlow();

        public native void setEpsilon(double d);

        public native void setGamma(double d);

        public native void setLambda(double d);

        public native void setNumIterations(int i);

        public native void setNumScales(int i);

        public native void setNumWarps(int i);

        public native void setScaleStep(double d);

        public native void setTau(double d);

        public native void setTheta(double d);

        public native void setUseInitialFlow(@Cast({"bool"}) boolean z);

        static {
            Loader.load();
        }

        public OpticalFlowDual_TVL1(Pointer p) {
            super(p);
        }
    }
}
