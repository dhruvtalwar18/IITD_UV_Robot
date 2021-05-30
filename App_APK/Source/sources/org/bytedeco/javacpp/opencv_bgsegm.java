package org.bytedeco.javacpp;

import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Const;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.annotation.NoOffset;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.opencv_video;
import org.bytedeco.javacpp.presets.opencv_core;

public class opencv_bgsegm extends org.bytedeco.javacpp.presets.opencv_bgsegm {
    public static final int LSBP_CAMERA_MOTION_COMPENSATION_LK = 1;
    public static final int LSBP_CAMERA_MOTION_COMPENSATION_NONE = 0;

    @Namespace("cv::bgsegm")
    @opencv_core.Ptr
    public static native BackgroundSubtractorCNT createBackgroundSubtractorCNT();

    @Namespace("cv::bgsegm")
    @opencv_core.Ptr
    public static native BackgroundSubtractorCNT createBackgroundSubtractorCNT(int i, @Cast({"bool"}) boolean z, int i2, @Cast({"bool"}) boolean z2);

    @Namespace("cv::bgsegm")
    @opencv_core.Ptr
    public static native BackgroundSubtractorGMG createBackgroundSubtractorGMG();

    @Namespace("cv::bgsegm")
    @opencv_core.Ptr
    public static native BackgroundSubtractorGMG createBackgroundSubtractorGMG(int i, double d);

    @Namespace("cv::bgsegm")
    @opencv_core.Ptr
    public static native BackgroundSubtractorGSOC createBackgroundSubtractorGSOC();

    @Namespace("cv::bgsegm")
    @opencv_core.Ptr
    public static native BackgroundSubtractorGSOC createBackgroundSubtractorGSOC(int i, int i2, float f, float f2, int i3, float f3, float f4, float f5, float f6, float f7, float f8);

    @Namespace("cv::bgsegm")
    @opencv_core.Ptr
    public static native BackgroundSubtractorLSBP createBackgroundSubtractorLSBP();

    @Namespace("cv::bgsegm")
    @opencv_core.Ptr
    public static native BackgroundSubtractorLSBP createBackgroundSubtractorLSBP(int i, int i2, int i3, float f, float f2, float f3, float f4, float f5, float f6, float f7, float f8, int i4, int i5);

    @Namespace("cv::bgsegm")
    @opencv_core.Ptr
    public static native BackgroundSubtractorMOG createBackgroundSubtractorMOG();

    @Namespace("cv::bgsegm")
    @opencv_core.Ptr
    public static native BackgroundSubtractorMOG createBackgroundSubtractorMOG(int i, int i2, double d, double d2);

    @Namespace("cv::bgsegm")
    @opencv_core.Ptr
    public static native SyntheticSequenceGenerator createSyntheticSequenceGenerator(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::bgsegm")
    @opencv_core.Ptr
    public static native SyntheticSequenceGenerator createSyntheticSequenceGenerator(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, double d, double d2, double d3, double d4);

    @Namespace("cv::bgsegm")
    @opencv_core.Ptr
    public static native SyntheticSequenceGenerator createSyntheticSequenceGenerator(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::bgsegm")
    @opencv_core.Ptr
    public static native SyntheticSequenceGenerator createSyntheticSequenceGenerator(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, double d, double d2, double d3, double d4);

    @Namespace("cv::bgsegm")
    @opencv_core.Ptr
    public static native SyntheticSequenceGenerator createSyntheticSequenceGenerator(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::bgsegm")
    @opencv_core.Ptr
    public static native SyntheticSequenceGenerator createSyntheticSequenceGenerator(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, double d, double d2, double d3, double d4);

    static {
        Loader.load();
    }

    @Namespace("cv::bgsegm")
    public static class BackgroundSubtractorMOG extends opencv_video.BackgroundSubtractor {
        public native double getBackgroundRatio();

        public native int getHistory();

        public native int getNMixtures();

        public native double getNoiseSigma();

        public native void setBackgroundRatio(double d);

        public native void setHistory(int i);

        public native void setNMixtures(int i);

        public native void setNoiseSigma(double d);

        static {
            Loader.load();
        }

        public BackgroundSubtractorMOG(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::bgsegm")
    public static class BackgroundSubtractorGMG extends opencv_video.BackgroundSubtractor {
        public native double getBackgroundPrior();

        public native double getDecisionThreshold();

        public native double getDefaultLearningRate();

        public native int getMaxFeatures();

        public native double getMaxVal();

        public native double getMinVal();

        public native int getNumFrames();

        public native int getQuantizationLevels();

        public native int getSmoothingRadius();

        @Cast({"bool"})
        public native boolean getUpdateBackgroundModel();

        public native void setBackgroundPrior(double d);

        public native void setDecisionThreshold(double d);

        public native void setDefaultLearningRate(double d);

        public native void setMaxFeatures(int i);

        public native void setMaxVal(double d);

        public native void setMinVal(double d);

        public native void setNumFrames(int i);

        public native void setQuantizationLevels(int i);

        public native void setSmoothingRadius(int i);

        public native void setUpdateBackgroundModel(@Cast({"bool"}) boolean z);

        static {
            Loader.load();
        }

        public BackgroundSubtractorGMG(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::bgsegm")
    public static class BackgroundSubtractorCNT extends opencv_video.BackgroundSubtractor {
        public native void apply(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void apply(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, double d);

        public native void apply(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void apply(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, double d);

        public native void apply(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void apply(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, double d);

        public native void getBackgroundImage(@ByVal opencv_core.GpuMat gpuMat);

        public native void getBackgroundImage(@ByVal opencv_core.Mat mat);

        public native void getBackgroundImage(@ByVal opencv_core.UMat uMat);

        @Cast({"bool"})
        public native boolean getIsParallel();

        public native int getMaxPixelStability();

        public native int getMinPixelStability();

        @Cast({"bool"})
        public native boolean getUseHistory();

        public native void setIsParallel(@Cast({"bool"}) boolean z);

        public native void setMaxPixelStability(int i);

        public native void setMinPixelStability(int i);

        public native void setUseHistory(@Cast({"bool"}) boolean z);

        static {
            Loader.load();
        }

        public BackgroundSubtractorCNT(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::bgsegm")
    public static class BackgroundSubtractorGSOC extends opencv_video.BackgroundSubtractor {
        public native void apply(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void apply(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, double d);

        public native void apply(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void apply(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, double d);

        public native void apply(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void apply(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, double d);

        public native void getBackgroundImage(@ByVal opencv_core.GpuMat gpuMat);

        public native void getBackgroundImage(@ByVal opencv_core.Mat mat);

        public native void getBackgroundImage(@ByVal opencv_core.UMat uMat);

        static {
            Loader.load();
        }

        public BackgroundSubtractorGSOC(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::bgsegm")
    public static class BackgroundSubtractorLSBP extends opencv_video.BackgroundSubtractor {
        public native void apply(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void apply(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, double d);

        public native void apply(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void apply(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, double d);

        public native void apply(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void apply(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, double d);

        public native void getBackgroundImage(@ByVal opencv_core.GpuMat gpuMat);

        public native void getBackgroundImage(@ByVal opencv_core.Mat mat);

        public native void getBackgroundImage(@ByVal opencv_core.UMat uMat);

        static {
            Loader.load();
        }

        public BackgroundSubtractorLSBP(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::bgsegm")
    public static class BackgroundSubtractorLSBPDesc extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        public static native void calcLocalSVDValues(@ByVal opencv_core.GpuMat gpuMat, @ByRef @Const opencv_core.Mat mat);

        public static native void calcLocalSVDValues(@ByVal opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2);

        public static native void calcLocalSVDValues(@ByVal opencv_core.UMat uMat, @ByRef @Const opencv_core.Mat mat);

        public static native void compute(@ByVal opencv_core.GpuMat gpuMat, @ByRef @Const opencv_core.Mat mat, @Cast({"const cv::Point2i*"}) opencv_core.Point point);

        public static native void compute(@ByVal opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2, @Cast({"const cv::Point2i*"}) opencv_core.Point point);

        public static native void compute(@ByVal opencv_core.UMat uMat, @ByRef @Const opencv_core.Mat mat, @Cast({"const cv::Point2i*"}) opencv_core.Point point);

        public static native void computeFromLocalSVDValues(@ByVal opencv_core.GpuMat gpuMat, @ByRef @Const opencv_core.Mat mat, @Cast({"const cv::Point2i*"}) opencv_core.Point point);

        public static native void computeFromLocalSVDValues(@ByVal opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2, @Cast({"const cv::Point2i*"}) opencv_core.Point point);

        public static native void computeFromLocalSVDValues(@ByVal opencv_core.UMat uMat, @ByRef @Const opencv_core.Mat mat, @Cast({"const cv::Point2i*"}) opencv_core.Point point);

        static {
            Loader.load();
        }

        public BackgroundSubtractorLSBPDesc() {
            super((Pointer) null);
            allocate();
        }

        public BackgroundSubtractorLSBPDesc(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public BackgroundSubtractorLSBPDesc(Pointer p) {
            super(p);
        }

        public BackgroundSubtractorLSBPDesc position(long position) {
            return (BackgroundSubtractorLSBPDesc) super.position(position);
        }
    }

    @Namespace("cv::bgsegm")
    @NoOffset
    public static class SyntheticSequenceGenerator extends opencv_core.Algorithm {
        private native void allocate(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, double d, double d2, double d3, double d4);

        private native void allocate(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, double d, double d2, double d3, double d4);

        private native void allocate(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, double d, double d2, double d3, double d4);

        public native void getNextFrame(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void getNextFrame(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void getNextFrame(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        static {
            Loader.load();
        }

        public SyntheticSequenceGenerator(Pointer p) {
            super(p);
        }

        public SyntheticSequenceGenerator(@ByVal opencv_core.Mat background, @ByVal opencv_core.Mat object, double amplitude, double wavelength, double wavespeed, double objspeed) {
            super((Pointer) null);
            allocate(background, object, amplitude, wavelength, wavespeed, objspeed);
        }

        public SyntheticSequenceGenerator(@ByVal opencv_core.UMat background, @ByVal opencv_core.UMat object, double amplitude, double wavelength, double wavespeed, double objspeed) {
            super((Pointer) null);
            allocate(background, object, amplitude, wavelength, wavespeed, objspeed);
        }

        public SyntheticSequenceGenerator(@ByVal opencv_core.GpuMat background, @ByVal opencv_core.GpuMat object, double amplitude, double wavelength, double wavespeed, double objspeed) {
            super((Pointer) null);
            allocate(background, object, amplitude, wavelength, wavespeed, objspeed);
        }
    }
}
