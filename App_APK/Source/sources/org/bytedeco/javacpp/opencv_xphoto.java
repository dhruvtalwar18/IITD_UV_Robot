package org.bytedeco.javacpp;

import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Const;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.opencv_photo;
import org.bytedeco.javacpp.presets.opencv_core;

public class opencv_xphoto extends org.bytedeco.javacpp.presets.opencv_xphoto {
    public static final int BM3D_STEP1 = 1;
    public static final int BM3D_STEP2 = 2;
    public static final int BM3D_STEPALL = 0;
    public static final int HAAR = 0;
    public static final int INPAINT_SHIFTMAP = 0;

    @Namespace("cv::xphoto")
    public static native void applyChannelGains(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, float f, float f2, float f3);

    @Namespace("cv::xphoto")
    public static native void applyChannelGains(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, float f, float f2, float f3);

    @Namespace("cv::xphoto")
    public static native void applyChannelGains(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, float f, float f2, float f3);

    @Namespace("cv::xphoto")
    public static native void bm3dDenoising(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::xphoto")
    public static native void bm3dDenoising(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, float f, int i, int i2, int i3, int i4, int i5, int i6, float f2, int i7, int i8, int i9);

    @Namespace("cv::xphoto")
    public static native void bm3dDenoising(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv::xphoto")
    public static native void bm3dDenoising(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, float f, int i, int i2, int i3, int i4, int i5, int i6, float f2, int i7, int i8, int i9);

    @Namespace("cv::xphoto")
    public static native void bm3dDenoising(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::xphoto")
    public static native void bm3dDenoising(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, float f, int i, int i2, int i3, int i4, int i5, int i6, float f2, int i7, int i8, int i9);

    @Namespace("cv::xphoto")
    public static native void bm3dDenoising(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv::xphoto")
    public static native void bm3dDenoising(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, float f, int i, int i2, int i3, int i4, int i5, int i6, float f2, int i7, int i8, int i9);

    @Namespace("cv::xphoto")
    public static native void bm3dDenoising(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::xphoto")
    public static native void bm3dDenoising(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, float f, int i, int i2, int i3, int i4, int i5, int i6, float f2, int i7, int i8, int i9);

    @Namespace("cv::xphoto")
    public static native void bm3dDenoising(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv::xphoto")
    public static native void bm3dDenoising(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, float f, int i, int i2, int i3, int i4, int i5, int i6, float f2, int i7, int i8, int i9);

    @Namespace("cv::xphoto")
    @opencv_core.Ptr
    public static native GrayworldWB createGrayworldWB();

    @Namespace("cv::xphoto")
    @opencv_core.Ptr
    public static native LearningBasedWB createLearningBasedWB();

    @Namespace("cv::xphoto")
    @opencv_core.Ptr
    public static native LearningBasedWB createLearningBasedWB(@opencv_core.Str String str);

    @Namespace("cv::xphoto")
    @opencv_core.Ptr
    public static native LearningBasedWB createLearningBasedWB(@opencv_core.Str BytePointer bytePointer);

    @Namespace("cv::xphoto")
    @opencv_core.Ptr
    public static native SimpleWB createSimpleWB();

    @Namespace("cv::xphoto")
    @opencv_core.Ptr
    public static native TonemapDurand createTonemapDurand();

    @Namespace("cv::xphoto")
    @opencv_core.Ptr
    public static native TonemapDurand createTonemapDurand(float f, float f2, float f3, float f4, float f5);

    @Namespace("cv::xphoto")
    public static native void dctDenoising(@ByRef @Const opencv_core.Mat mat, @ByRef opencv_core.Mat mat2, double d);

    @Namespace("cv::xphoto")
    public static native void dctDenoising(@ByRef @Const opencv_core.Mat mat, @ByRef opencv_core.Mat mat2, double d, int i);

    @Namespace("cv::xphoto")
    public static native void inpaint(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2, @ByRef opencv_core.Mat mat3, int i);

    @Namespace("cv::xphoto")
    public static native void oilPainting(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, int i2);

    @Namespace("cv::xphoto")
    public static native void oilPainting(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, int i2, int i3);

    @Namespace("cv::xphoto")
    public static native void oilPainting(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, int i2);

    @Namespace("cv::xphoto")
    public static native void oilPainting(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, int i2, int i3);

    @Namespace("cv::xphoto")
    public static native void oilPainting(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, int i2);

    @Namespace("cv::xphoto")
    public static native void oilPainting(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, int i2, int i3);

    static {
        Loader.load();
    }

    @Namespace("cv::xphoto")
    public static class WhiteBalancer extends opencv_core.Algorithm {
        public native void balanceWhite(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void balanceWhite(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void balanceWhite(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        static {
            Loader.load();
        }

        public WhiteBalancer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::xphoto")
    public static class SimpleWB extends WhiteBalancer {
        public native float getInputMax();

        public native float getInputMin();

        public native float getOutputMax();

        public native float getOutputMin();

        public native float getP();

        public native void setInputMax(float f);

        public native void setInputMin(float f);

        public native void setOutputMax(float f);

        public native void setOutputMin(float f);

        public native void setP(float f);

        static {
            Loader.load();
        }

        public SimpleWB(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::xphoto")
    public static class GrayworldWB extends WhiteBalancer {
        public native float getSaturationThreshold();

        public native void setSaturationThreshold(float f);

        static {
            Loader.load();
        }

        public GrayworldWB(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::xphoto")
    public static class LearningBasedWB extends WhiteBalancer {
        public native void extractSimpleFeatures(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void extractSimpleFeatures(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void extractSimpleFeatures(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native int getHistBinNum();

        public native int getRangeMaxVal();

        public native float getSaturationThreshold();

        public native void setHistBinNum(int i);

        public native void setRangeMaxVal(int i);

        public native void setSaturationThreshold(float f);

        static {
            Loader.load();
        }

        public LearningBasedWB(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::xphoto")
    public static class TonemapDurand extends opencv_photo.Tonemap {
        public native float getContrast();

        public native float getSaturation();

        public native float getSigmaColor();

        public native float getSigmaSpace();

        public native void setContrast(float f);

        public native void setSaturation(float f);

        public native void setSigmaColor(float f);

        public native void setSigmaSpace(float f);

        static {
            Loader.load();
        }

        public TonemapDurand(Pointer p) {
            super(p);
        }
    }
}
