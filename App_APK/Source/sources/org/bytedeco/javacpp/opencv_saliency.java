package org.bytedeco.javacpp;

import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Const;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.annotation.NoOffset;
import org.bytedeco.javacpp.annotation.StdVector;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.presets.opencv_core;

public class opencv_saliency extends org.bytedeco.javacpp.presets.opencv_saliency {
    static {
        Loader.load();
    }

    @Namespace("cv::saliency")
    @NoOffset
    public static class Saliency extends opencv_core.Algorithm {
        @Cast({"bool"})
        public native boolean computeSaliency(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        @Cast({"bool"})
        public native boolean computeSaliency(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        @Cast({"bool"})
        public native boolean computeSaliency(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        static {
            Loader.load();
        }

        public Saliency(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::saliency")
    public static class StaticSaliency extends Saliency {
        @Cast({"bool"})
        public native boolean computeBinaryMap(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        @Cast({"bool"})
        public native boolean computeBinaryMap(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        @Cast({"bool"})
        public native boolean computeBinaryMap(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        static {
            Loader.load();
        }

        public StaticSaliency(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::saliency")
    public static class MotionSaliency extends Saliency {
        static {
            Loader.load();
        }

        public MotionSaliency(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::saliency")
    public static class Objectness extends Saliency {
        static {
            Loader.load();
        }

        public Objectness(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::saliency")
    @NoOffset
    public static class StaticSaliencySpectralResidual extends StaticSaliency {
        private native void allocate();

        private native void allocateArray(long j);

        @opencv_core.Ptr
        public static native StaticSaliencySpectralResidual create();

        @Cast({"bool"})
        public native boolean computeSaliency(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        @Cast({"bool"})
        public native boolean computeSaliency(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        @Cast({"bool"})
        public native boolean computeSaliency(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native int getImageHeight();

        public native int getImageWidth();

        public native void read(@ByRef @Const opencv_core.FileNode fileNode);

        public native void setImageHeight(int i);

        public native void setImageWidth(int i);

        public native void write(@ByRef opencv_core.FileStorage fileStorage);

        static {
            Loader.load();
        }

        public StaticSaliencySpectralResidual(Pointer p) {
            super(p);
        }

        public StaticSaliencySpectralResidual(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public StaticSaliencySpectralResidual position(long position) {
            return (StaticSaliencySpectralResidual) super.position(position);
        }

        public StaticSaliencySpectralResidual() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::saliency")
    public static class StaticSaliencyFineGrained extends StaticSaliency {
        private native void allocate();

        private native void allocateArray(long j);

        @opencv_core.Ptr
        public static native StaticSaliencyFineGrained create();

        @Cast({"bool"})
        public native boolean computeSaliency(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        @Cast({"bool"})
        public native boolean computeSaliency(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        @Cast({"bool"})
        public native boolean computeSaliency(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        static {
            Loader.load();
        }

        public StaticSaliencyFineGrained(Pointer p) {
            super(p);
        }

        public StaticSaliencyFineGrained(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public StaticSaliencyFineGrained position(long position) {
            return (StaticSaliencyFineGrained) super.position(position);
        }

        public StaticSaliencyFineGrained() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::saliency")
    @NoOffset
    public static class MotionSaliencyBinWangApr2014 extends MotionSaliency {
        private native void allocate();

        private native void allocateArray(long j);

        @opencv_core.Ptr
        public static native MotionSaliencyBinWangApr2014 create();

        @Cast({"bool"})
        public native boolean computeSaliency(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        @Cast({"bool"})
        public native boolean computeSaliency(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        @Cast({"bool"})
        public native boolean computeSaliency(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native int getImageHeight();

        public native int getImageWidth();

        @Cast({"bool"})
        public native boolean init();

        public native void setImageHeight(int i);

        public native void setImageWidth(int i);

        public native void setImagesize(int i, int i2);

        static {
            Loader.load();
        }

        public MotionSaliencyBinWangApr2014(Pointer p) {
            super(p);
        }

        public MotionSaliencyBinWangApr2014(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public MotionSaliencyBinWangApr2014 position(long position) {
            return (MotionSaliencyBinWangApr2014) super.position(position);
        }

        public MotionSaliencyBinWangApr2014() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::saliency")
    @NoOffset
    public static class ObjectnessBING extends Objectness {
        private native void allocate();

        private native void allocateArray(long j);

        @opencv_core.Ptr
        public static native ObjectnessBING create();

        @Cast({"bool"})
        public native boolean computeSaliency(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        @Cast({"bool"})
        public native boolean computeSaliency(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        @Cast({"bool"})
        public native boolean computeSaliency(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native double getBase();

        public native int getNSS();

        public native int getW();

        @StdVector
        public native FloatPointer getobjectnessValues();

        public native void read();

        public native void setBBResDir(@opencv_core.Str String str);

        public native void setBBResDir(@opencv_core.Str BytePointer bytePointer);

        public native void setBase(double d);

        public native void setNSS(int i);

        public native void setTrainingPath(@opencv_core.Str String str);

        public native void setTrainingPath(@opencv_core.Str BytePointer bytePointer);

        public native void setW(int i);

        public native void write();

        static {
            Loader.load();
        }

        public ObjectnessBING(Pointer p) {
            super(p);
        }

        public ObjectnessBING(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public ObjectnessBING position(long position) {
            return (ObjectnessBING) super.position(position);
        }

        public ObjectnessBING() {
            super((Pointer) null);
            allocate();
        }
    }
}
