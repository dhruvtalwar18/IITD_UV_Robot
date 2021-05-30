package org.bytedeco.javacpp;

import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Const;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.annotation.NoOffset;
import org.bytedeco.javacpp.annotation.Opaque;
import org.bytedeco.javacpp.annotation.StdVector;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.presets.opencv_core;

public class opencv_img_hash extends org.bytedeco.javacpp.presets.opencv_img_hash {
    public static final int BLOCK_MEAN_HASH_MODE_0 = 0;
    public static final int BLOCK_MEAN_HASH_MODE_1 = 1;

    @Namespace("cv::img_hash")
    public static native void averageHash(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::img_hash")
    public static native void averageHash(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::img_hash")
    public static native void averageHash(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::img_hash")
    public static native void blockMeanHash(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::img_hash")
    public static native void blockMeanHash(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i);

    @Namespace("cv::img_hash")
    public static native void blockMeanHash(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::img_hash")
    public static native void blockMeanHash(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i);

    @Namespace("cv::img_hash")
    public static native void blockMeanHash(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::img_hash")
    public static native void blockMeanHash(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i);

    @Namespace("cv::img_hash")
    public static native void colorMomentHash(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::img_hash")
    public static native void colorMomentHash(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::img_hash")
    public static native void colorMomentHash(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::img_hash")
    public static native void marrHildrethHash(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::img_hash")
    public static native void marrHildrethHash(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, float f, float f2);

    @Namespace("cv::img_hash")
    public static native void marrHildrethHash(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::img_hash")
    public static native void marrHildrethHash(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, float f, float f2);

    @Namespace("cv::img_hash")
    public static native void marrHildrethHash(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::img_hash")
    public static native void marrHildrethHash(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, float f, float f2);

    @Namespace("cv::img_hash")
    public static native void pHash(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::img_hash")
    public static native void pHash(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::img_hash")
    public static native void pHash(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::img_hash")
    public static native void radialVarianceHash(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::img_hash")
    public static native void radialVarianceHash(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, double d, int i);

    @Namespace("cv::img_hash")
    public static native void radialVarianceHash(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::img_hash")
    public static native void radialVarianceHash(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, double d, int i);

    @Namespace("cv::img_hash")
    public static native void radialVarianceHash(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::img_hash")
    public static native void radialVarianceHash(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, double d, int i);

    static {
        Loader.load();
    }

    @Namespace("cv::img_hash")
    @NoOffset
    public static class ImgHashBase extends opencv_core.Algorithm {
        public native double compare(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native double compare(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native double compare(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void compute(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void compute(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void compute(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        static {
            Loader.load();
        }

        public ImgHashBase(Pointer p) {
            super(p);
        }

        @Opaque
        public static class ImgHashImpl extends Pointer {
            public ImgHashImpl() {
                super((Pointer) null);
            }

            public ImgHashImpl(Pointer p) {
                super(p);
            }
        }
    }

    @Namespace("cv::img_hash")
    public static class AverageHash extends ImgHashBase {
        @opencv_core.Ptr
        public static native AverageHash create();

        static {
            Loader.load();
        }

        public AverageHash(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::img_hash")
    public static class BlockMeanHash extends ImgHashBase {
        @opencv_core.Ptr
        public static native BlockMeanHash create();

        @opencv_core.Ptr
        public static native BlockMeanHash create(int i);

        @StdVector
        public native DoublePointer getMean();

        public native void setMode(int i);

        static {
            Loader.load();
        }

        public BlockMeanHash(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::img_hash")
    public static class ColorMomentHash extends ImgHashBase {
        @opencv_core.Ptr
        public static native ColorMomentHash create();

        static {
            Loader.load();
        }

        public ColorMomentHash(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::img_hash")
    public static class MarrHildrethHash extends ImgHashBase {
        @opencv_core.Ptr
        public static native MarrHildrethHash create();

        @opencv_core.Ptr
        public static native MarrHildrethHash create(float f, float f2);

        public native float getAlpha();

        public native float getScale();

        public native void setKernelParam(float f, float f2);

        static {
            Loader.load();
        }

        public MarrHildrethHash(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::img_hash")
    public static class PHash extends ImgHashBase {
        @opencv_core.Ptr
        public static native PHash create();

        static {
            Loader.load();
        }

        public PHash(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::img_hash")
    public static class RadialVarianceHash extends ImgHashBase {
        @opencv_core.Ptr
        public static native RadialVarianceHash create();

        @opencv_core.Ptr
        public static native RadialVarianceHash create(double d, int i);

        @StdVector
        public native DoublePointer getFeatures();

        @ByVal
        public native opencv_core.Mat getHash();

        public native int getNumOfAngleLine();

        @ByVal
        public native opencv_core.Mat getPixPerLine(@ByRef @Const opencv_core.Mat mat);

        @ByVal
        public native opencv_core.Mat getProjection();

        public native double getSigma();

        public native void setNumOfAngleLine(int i);

        public native void setSigma(double d);

        static {
            Loader.load();
        }

        public RadialVarianceHash(Pointer p) {
            super(p);
        }
    }
}
