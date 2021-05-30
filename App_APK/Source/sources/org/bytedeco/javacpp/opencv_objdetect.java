package org.bytedeco.javacpp;

import java.nio.DoubleBuffer;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Const;
import org.bytedeco.javacpp.annotation.Name;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.annotation.NoOffset;
import org.bytedeco.javacpp.annotation.Opaque;
import org.bytedeco.javacpp.annotation.StdString;
import org.bytedeco.javacpp.annotation.StdVector;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.presets.opencv_core;

public class opencv_objdetect extends org.bytedeco.javacpp.helper.opencv_objdetect {
    public static final int CASCADE_DO_CANNY_PRUNING = 1;
    public static final int CASCADE_DO_ROUGH_SEARCH = 8;
    public static final int CASCADE_FIND_BIGGEST_OBJECT = 4;
    public static final int CASCADE_SCALE_IMAGE = 2;

    @Namespace("cv")
    @opencv_core.Ptr
    public static native BaseCascadeClassifier.MaskGenerator createFaceDetectionMaskGenerator();

    @Namespace("cv")
    public static native void groupRectangles(@ByRef opencv_core.RectVector rectVector, int i);

    @Namespace("cv")
    public static native void groupRectangles(@ByRef opencv_core.RectVector rectVector, int i, double d);

    @Namespace("cv")
    public static native void groupRectangles(@ByRef opencv_core.RectVector rectVector, int i, double d, @StdVector IntBuffer intBuffer, @StdVector DoubleBuffer doubleBuffer);

    @Namespace("cv")
    public static native void groupRectangles(@ByRef opencv_core.RectVector rectVector, int i, double d, @StdVector IntPointer intPointer, @StdVector DoublePointer doublePointer);

    @Namespace("cv")
    public static native void groupRectangles(@ByRef opencv_core.RectVector rectVector, int i, double d, @StdVector int[] iArr, @StdVector double[] dArr);

    @Namespace("cv")
    public static native void groupRectangles(@ByRef opencv_core.RectVector rectVector, @StdVector IntBuffer intBuffer, int i);

    @Namespace("cv")
    public static native void groupRectangles(@ByRef opencv_core.RectVector rectVector, @StdVector IntBuffer intBuffer, int i, double d);

    @Namespace("cv")
    public static native void groupRectangles(@ByRef opencv_core.RectVector rectVector, @StdVector IntBuffer intBuffer, @StdVector DoubleBuffer doubleBuffer, int i);

    @Namespace("cv")
    public static native void groupRectangles(@ByRef opencv_core.RectVector rectVector, @StdVector IntBuffer intBuffer, @StdVector DoubleBuffer doubleBuffer, int i, double d);

    @Namespace("cv")
    public static native void groupRectangles(@ByRef opencv_core.RectVector rectVector, @StdVector IntPointer intPointer, int i);

    @Namespace("cv")
    public static native void groupRectangles(@ByRef opencv_core.RectVector rectVector, @StdVector IntPointer intPointer, int i, double d);

    @Namespace("cv")
    public static native void groupRectangles(@ByRef opencv_core.RectVector rectVector, @StdVector IntPointer intPointer, @StdVector DoublePointer doublePointer, int i);

    @Namespace("cv")
    public static native void groupRectangles(@ByRef opencv_core.RectVector rectVector, @StdVector IntPointer intPointer, @StdVector DoublePointer doublePointer, int i, double d);

    @Namespace("cv")
    public static native void groupRectangles(@ByRef opencv_core.RectVector rectVector, @StdVector int[] iArr, int i);

    @Namespace("cv")
    public static native void groupRectangles(@ByRef opencv_core.RectVector rectVector, @StdVector int[] iArr, int i, double d);

    @Namespace("cv")
    public static native void groupRectangles(@ByRef opencv_core.RectVector rectVector, @StdVector int[] iArr, @StdVector double[] dArr, int i);

    @Namespace("cv")
    public static native void groupRectangles(@ByRef opencv_core.RectVector rectVector, @StdVector int[] iArr, @StdVector double[] dArr, int i, double d);

    @Namespace("cv")
    public static native void groupRectangles_meanshift(@ByRef opencv_core.RectVector rectVector, @StdVector DoubleBuffer doubleBuffer, @StdVector DoubleBuffer doubleBuffer2);

    @Namespace("cv")
    public static native void groupRectangles_meanshift(@ByRef opencv_core.RectVector rectVector, @StdVector DoubleBuffer doubleBuffer, @StdVector DoubleBuffer doubleBuffer2, double d, @ByVal(nullValue = "cv::Size(64, 128)") opencv_core.Size size);

    @Namespace("cv")
    public static native void groupRectangles_meanshift(@ByRef opencv_core.RectVector rectVector, @StdVector DoublePointer doublePointer, @StdVector DoublePointer doublePointer2);

    @Namespace("cv")
    public static native void groupRectangles_meanshift(@ByRef opencv_core.RectVector rectVector, @StdVector DoublePointer doublePointer, @StdVector DoublePointer doublePointer2, double d, @ByVal(nullValue = "cv::Size(64, 128)") opencv_core.Size size);

    @Namespace("cv")
    public static native void groupRectangles_meanshift(@ByRef opencv_core.RectVector rectVector, @StdVector double[] dArr, @StdVector double[] dArr2);

    @Namespace("cv")
    public static native void groupRectangles_meanshift(@ByRef opencv_core.RectVector rectVector, @StdVector double[] dArr, @StdVector double[] dArr2, double d, @ByVal(nullValue = "cv::Size(64, 128)") opencv_core.Size size);

    static {
        Loader.load();
    }

    @Opaque
    public static class CvHaarClassifierCascade extends Pointer {
        public CvHaarClassifierCascade() {
            super((Pointer) null);
        }

        public CvHaarClassifierCascade(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class SimilarRects extends Pointer {
        private native void allocate(double d);

        @Cast({"bool"})
        @Name({"operator ()"})
        public native boolean apply(@ByRef @Const opencv_core.Rect rect, @ByRef @Const opencv_core.Rect rect2);

        public native double eps();

        public native SimilarRects eps(double d);

        static {
            Loader.load();
        }

        public SimilarRects(Pointer p) {
            super(p);
        }

        public SimilarRects(double _eps) {
            super((Pointer) null);
            allocate(_eps);
        }
    }

    @Namespace("cv")
    public static class BaseCascadeClassifier extends opencv_core.Algorithm {
        public native void detectMultiScale(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, double d, int i, int i2, @ByVal opencv_core.Size size, @ByVal opencv_core.Size size2);

        public native void detectMultiScale(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector IntBuffer intBuffer, double d, int i, int i2, @ByVal opencv_core.Size size, @ByVal opencv_core.Size size2);

        public native void detectMultiScale(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector IntBuffer intBuffer, @StdVector DoubleBuffer doubleBuffer, double d, int i, int i2, @ByVal opencv_core.Size size, @ByVal opencv_core.Size size2, @Cast({"bool"}) boolean z);

        public native void detectMultiScale(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector IntPointer intPointer, double d, int i, int i2, @ByVal opencv_core.Size size, @ByVal opencv_core.Size size2);

        public native void detectMultiScale(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector IntPointer intPointer, @StdVector DoublePointer doublePointer, double d, int i, int i2, @ByVal opencv_core.Size size, @ByVal opencv_core.Size size2, @Cast({"bool"}) boolean z);

        public native void detectMultiScale(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector int[] iArr, double d, int i, int i2, @ByVal opencv_core.Size size, @ByVal opencv_core.Size size2);

        public native void detectMultiScale(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector int[] iArr, @StdVector double[] dArr, double d, int i, int i2, @ByVal opencv_core.Size size, @ByVal opencv_core.Size size2, @Cast({"bool"}) boolean z);

        public native void detectMultiScale(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, double d, int i, int i2, @ByVal opencv_core.Size size, @ByVal opencv_core.Size size2);

        public native void detectMultiScale(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector IntBuffer intBuffer, double d, int i, int i2, @ByVal opencv_core.Size size, @ByVal opencv_core.Size size2);

        public native void detectMultiScale(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector IntBuffer intBuffer, @StdVector DoubleBuffer doubleBuffer, double d, int i, int i2, @ByVal opencv_core.Size size, @ByVal opencv_core.Size size2, @Cast({"bool"}) boolean z);

        public native void detectMultiScale(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector IntPointer intPointer, double d, int i, int i2, @ByVal opencv_core.Size size, @ByVal opencv_core.Size size2);

        public native void detectMultiScale(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector IntPointer intPointer, @StdVector DoublePointer doublePointer, double d, int i, int i2, @ByVal opencv_core.Size size, @ByVal opencv_core.Size size2, @Cast({"bool"}) boolean z);

        public native void detectMultiScale(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector int[] iArr, double d, int i, int i2, @ByVal opencv_core.Size size, @ByVal opencv_core.Size size2);

        public native void detectMultiScale(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector int[] iArr, @StdVector double[] dArr, double d, int i, int i2, @ByVal opencv_core.Size size, @ByVal opencv_core.Size size2, @Cast({"bool"}) boolean z);

        public native void detectMultiScale(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, double d, int i, int i2, @ByVal opencv_core.Size size, @ByVal opencv_core.Size size2);

        public native void detectMultiScale(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector IntBuffer intBuffer, double d, int i, int i2, @ByVal opencv_core.Size size, @ByVal opencv_core.Size size2);

        public native void detectMultiScale(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector IntBuffer intBuffer, @StdVector DoubleBuffer doubleBuffer, double d, int i, int i2, @ByVal opencv_core.Size size, @ByVal opencv_core.Size size2, @Cast({"bool"}) boolean z);

        public native void detectMultiScale(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector IntPointer intPointer, double d, int i, int i2, @ByVal opencv_core.Size size, @ByVal opencv_core.Size size2);

        public native void detectMultiScale(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector IntPointer intPointer, @StdVector DoublePointer doublePointer, double d, int i, int i2, @ByVal opencv_core.Size size, @ByVal opencv_core.Size size2, @Cast({"bool"}) boolean z);

        public native void detectMultiScale(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector int[] iArr, double d, int i, int i2, @ByVal opencv_core.Size size, @ByVal opencv_core.Size size2);

        public native void detectMultiScale(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector int[] iArr, @StdVector double[] dArr, double d, int i, int i2, @ByVal opencv_core.Size size, @ByVal opencv_core.Size size2, @Cast({"bool"}) boolean z);

        @Cast({"bool"})
        public native boolean empty();

        public native int getFeatureType();

        @opencv_core.Ptr
        public native MaskGenerator getMaskGenerator();

        public native Pointer getOldCascade();

        @ByVal
        public native opencv_core.Size getOriginalWindowSize();

        @Cast({"bool"})
        public native boolean isOldFormatCascade();

        @Cast({"bool"})
        public native boolean load(@opencv_core.Str String str);

        @Cast({"bool"})
        public native boolean load(@opencv_core.Str BytePointer bytePointer);

        public native void setMaskGenerator(@opencv_core.Ptr MaskGenerator maskGenerator);

        static {
            Loader.load();
        }

        public BaseCascadeClassifier(Pointer p) {
            super(p);
        }

        public static class MaskGenerator extends Pointer {
            @ByVal
            public native opencv_core.Mat generateMask(@ByRef @Const opencv_core.Mat mat);

            public native void initializeMask(@ByRef @Const opencv_core.Mat mat);

            static {
                Loader.load();
            }

            public MaskGenerator(Pointer p) {
                super(p);
            }
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class CascadeClassifier extends Pointer {
        private native void allocate();

        private native void allocate(@opencv_core.Str String str);

        private native void allocate(@opencv_core.Str BytePointer bytePointer);

        private native void allocateArray(long j);

        @Cast({"bool"})
        public static native boolean convert(@opencv_core.Str String str, @opencv_core.Str String str2);

        @Cast({"bool"})
        public static native boolean convert(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

        @opencv_core.Ptr
        public native BaseCascadeClassifier cc();

        public native CascadeClassifier cc(BaseCascadeClassifier baseCascadeClassifier);

        public native void detectMultiScale(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector);

        public native void detectMultiScale(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, double d, int i, int i2, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2);

        public native void detectMultiScale(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector);

        public native void detectMultiScale(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, double d, int i, int i2, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2);

        public native void detectMultiScale(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector);

        public native void detectMultiScale(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, double d, int i, int i2, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2);

        @Name({"detectMultiScale"})
        public native void detectMultiScale2(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector IntBuffer intBuffer);

        @Name({"detectMultiScale"})
        public native void detectMultiScale2(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector IntBuffer intBuffer, double d, int i, int i2, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2);

        @Name({"detectMultiScale"})
        public native void detectMultiScale2(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector IntPointer intPointer);

        @Name({"detectMultiScale"})
        public native void detectMultiScale2(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector IntPointer intPointer, double d, int i, int i2, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2);

        @Name({"detectMultiScale"})
        public native void detectMultiScale2(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector int[] iArr);

        @Name({"detectMultiScale"})
        public native void detectMultiScale2(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector int[] iArr, double d, int i, int i2, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2);

        @Name({"detectMultiScale"})
        public native void detectMultiScale2(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector IntBuffer intBuffer);

        @Name({"detectMultiScale"})
        public native void detectMultiScale2(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector IntBuffer intBuffer, double d, int i, int i2, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2);

        @Name({"detectMultiScale"})
        public native void detectMultiScale2(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector IntPointer intPointer);

        @Name({"detectMultiScale"})
        public native void detectMultiScale2(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector IntPointer intPointer, double d, int i, int i2, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2);

        @Name({"detectMultiScale"})
        public native void detectMultiScale2(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector int[] iArr);

        @Name({"detectMultiScale"})
        public native void detectMultiScale2(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector int[] iArr, double d, int i, int i2, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2);

        @Name({"detectMultiScale"})
        public native void detectMultiScale2(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector IntBuffer intBuffer);

        @Name({"detectMultiScale"})
        public native void detectMultiScale2(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector IntBuffer intBuffer, double d, int i, int i2, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2);

        @Name({"detectMultiScale"})
        public native void detectMultiScale2(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector IntPointer intPointer);

        @Name({"detectMultiScale"})
        public native void detectMultiScale2(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector IntPointer intPointer, double d, int i, int i2, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2);

        @Name({"detectMultiScale"})
        public native void detectMultiScale2(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector int[] iArr);

        @Name({"detectMultiScale"})
        public native void detectMultiScale2(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector int[] iArr, double d, int i, int i2, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2);

        @Name({"detectMultiScale"})
        public native void detectMultiScale3(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector IntBuffer intBuffer, @StdVector DoubleBuffer doubleBuffer);

        @Name({"detectMultiScale"})
        public native void detectMultiScale3(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector IntBuffer intBuffer, @StdVector DoubleBuffer doubleBuffer, double d, int i, int i2, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @Cast({"bool"}) boolean z);

        @Name({"detectMultiScale"})
        public native void detectMultiScale3(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector IntPointer intPointer, @StdVector DoublePointer doublePointer);

        @Name({"detectMultiScale"})
        public native void detectMultiScale3(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector IntPointer intPointer, @StdVector DoublePointer doublePointer, double d, int i, int i2, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @Cast({"bool"}) boolean z);

        @Name({"detectMultiScale"})
        public native void detectMultiScale3(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector int[] iArr, @StdVector double[] dArr);

        @Name({"detectMultiScale"})
        public native void detectMultiScale3(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector int[] iArr, @StdVector double[] dArr, double d, int i, int i2, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @Cast({"bool"}) boolean z);

        @Name({"detectMultiScale"})
        public native void detectMultiScale3(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector IntBuffer intBuffer, @StdVector DoubleBuffer doubleBuffer);

        @Name({"detectMultiScale"})
        public native void detectMultiScale3(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector IntBuffer intBuffer, @StdVector DoubleBuffer doubleBuffer, double d, int i, int i2, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @Cast({"bool"}) boolean z);

        @Name({"detectMultiScale"})
        public native void detectMultiScale3(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector IntPointer intPointer, @StdVector DoublePointer doublePointer);

        @Name({"detectMultiScale"})
        public native void detectMultiScale3(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector IntPointer intPointer, @StdVector DoublePointer doublePointer, double d, int i, int i2, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @Cast({"bool"}) boolean z);

        @Name({"detectMultiScale"})
        public native void detectMultiScale3(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector int[] iArr, @StdVector double[] dArr);

        @Name({"detectMultiScale"})
        public native void detectMultiScale3(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector int[] iArr, @StdVector double[] dArr, double d, int i, int i2, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @Cast({"bool"}) boolean z);

        @Name({"detectMultiScale"})
        public native void detectMultiScale3(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector IntBuffer intBuffer, @StdVector DoubleBuffer doubleBuffer);

        @Name({"detectMultiScale"})
        public native void detectMultiScale3(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector IntBuffer intBuffer, @StdVector DoubleBuffer doubleBuffer, double d, int i, int i2, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @Cast({"bool"}) boolean z);

        @Name({"detectMultiScale"})
        public native void detectMultiScale3(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector IntPointer intPointer, @StdVector DoublePointer doublePointer);

        @Name({"detectMultiScale"})
        public native void detectMultiScale3(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector IntPointer intPointer, @StdVector DoublePointer doublePointer, double d, int i, int i2, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @Cast({"bool"}) boolean z);

        @Name({"detectMultiScale"})
        public native void detectMultiScale3(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector int[] iArr, @StdVector double[] dArr);

        @Name({"detectMultiScale"})
        public native void detectMultiScale3(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector int[] iArr, @StdVector double[] dArr, double d, int i, int i2, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @Cast({"bool"}) boolean z);

        @Cast({"bool"})
        public native boolean empty();

        public native int getFeatureType();

        @opencv_core.Ptr
        public native BaseCascadeClassifier.MaskGenerator getMaskGenerator();

        public native Pointer getOldCascade();

        @ByVal
        public native opencv_core.Size getOriginalWindowSize();

        @Cast({"bool"})
        public native boolean isOldFormatCascade();

        @Cast({"bool"})
        public native boolean load(@opencv_core.Str String str);

        @Cast({"bool"})
        public native boolean load(@opencv_core.Str BytePointer bytePointer);

        @Cast({"bool"})
        public native boolean read(@ByRef @Const opencv_core.FileNode fileNode);

        public native void setMaskGenerator(@opencv_core.Ptr BaseCascadeClassifier.MaskGenerator maskGenerator);

        static {
            Loader.load();
        }

        public CascadeClassifier(Pointer p) {
            super(p);
        }

        public CascadeClassifier(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CascadeClassifier position(long position) {
            return (CascadeClassifier) super.position(position);
        }

        public CascadeClassifier() {
            super((Pointer) null);
            allocate();
        }

        public CascadeClassifier(@opencv_core.Str BytePointer filename) {
            super((Pointer) null);
            allocate(filename);
        }

        public CascadeClassifier(@opencv_core.Str String filename) {
            super((Pointer) null);
            allocate(filename);
        }
    }

    @Namespace("cv")
    public static class DetectionROI extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        @StdVector
        public native DoublePointer confidences();

        public native DetectionROI confidences(DoublePointer doublePointer);

        @ByRef
        public native opencv_core.PointVector locations();

        public native DetectionROI locations(opencv_core.PointVector pointVector);

        public native double scale();

        public native DetectionROI scale(double d);

        static {
            Loader.load();
        }

        public DetectionROI() {
            super((Pointer) null);
            allocate();
        }

        public DetectionROI(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public DetectionROI(Pointer p) {
            super(p);
        }

        public DetectionROI position(long position) {
            return (DetectionROI) super.position(position);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class HOGDescriptor extends Pointer {
        public static final int DEFAULT_NLEVELS = 64;
        public static final int DESCR_FORMAT_COL_BY_COL = 0;
        public static final int DESCR_FORMAT_ROW_BY_ROW = 1;
        public static final int L2Hys = 0;

        private native void allocate();

        private native void allocate(@opencv_core.Str String str);

        private native void allocate(@opencv_core.Str BytePointer bytePointer);

        private native void allocate(@ByVal opencv_core.Size size, @ByVal opencv_core.Size size2, @ByVal opencv_core.Size size3, @ByVal opencv_core.Size size4, int i);

        private native void allocate(@ByVal opencv_core.Size size, @ByVal opencv_core.Size size2, @ByVal opencv_core.Size size3, @ByVal opencv_core.Size size4, int i, int i2, double d, @Cast({"cv::HOGDescriptor::HistogramNormType"}) int i3, double d2, @Cast({"bool"}) boolean z, int i4, @Cast({"bool"}) boolean z2);

        private native void allocate(@ByRef @Const HOGDescriptor hOGDescriptor);

        private native void allocateArray(long j);

        @StdVector
        public static native FloatPointer getDaimlerPeopleDetector();

        @StdVector
        public static native FloatPointer getDefaultPeopleDetector();

        public native double L2HysThreshold();

        public native HOGDescriptor L2HysThreshold(double d);

        @ByRef
        public native opencv_core.Size blockSize();

        public native HOGDescriptor blockSize(opencv_core.Size size);

        @ByRef
        public native opencv_core.Size blockStride();

        public native HOGDescriptor blockStride(opencv_core.Size size);

        @ByRef
        public native opencv_core.Size cellSize();

        public native HOGDescriptor cellSize(opencv_core.Size size);

        @Cast({"bool"})
        public native boolean checkDetectorSize();

        public native void compute(@ByVal opencv_core.GpuMat gpuMat, @StdVector FloatBuffer floatBuffer);

        public native void compute(@ByVal opencv_core.GpuMat gpuMat, @StdVector FloatBuffer floatBuffer, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @ByRef(nullValue = "std::vector<cv::Point>()") @Const opencv_core.PointVector pointVector);

        public native void compute(@ByVal opencv_core.GpuMat gpuMat, @StdVector FloatPointer floatPointer);

        public native void compute(@ByVal opencv_core.GpuMat gpuMat, @StdVector FloatPointer floatPointer, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @ByRef(nullValue = "std::vector<cv::Point>()") @Const opencv_core.PointVector pointVector);

        public native void compute(@ByVal opencv_core.GpuMat gpuMat, @StdVector float[] fArr);

        public native void compute(@ByVal opencv_core.GpuMat gpuMat, @StdVector float[] fArr, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @ByRef(nullValue = "std::vector<cv::Point>()") @Const opencv_core.PointVector pointVector);

        public native void compute(@ByVal opencv_core.Mat mat, @StdVector FloatBuffer floatBuffer);

        public native void compute(@ByVal opencv_core.Mat mat, @StdVector FloatBuffer floatBuffer, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @ByRef(nullValue = "std::vector<cv::Point>()") @Const opencv_core.PointVector pointVector);

        public native void compute(@ByVal opencv_core.Mat mat, @StdVector FloatPointer floatPointer);

        public native void compute(@ByVal opencv_core.Mat mat, @StdVector FloatPointer floatPointer, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @ByRef(nullValue = "std::vector<cv::Point>()") @Const opencv_core.PointVector pointVector);

        public native void compute(@ByVal opencv_core.Mat mat, @StdVector float[] fArr);

        public native void compute(@ByVal opencv_core.Mat mat, @StdVector float[] fArr, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @ByRef(nullValue = "std::vector<cv::Point>()") @Const opencv_core.PointVector pointVector);

        public native void compute(@ByVal opencv_core.UMat uMat, @StdVector FloatBuffer floatBuffer);

        public native void compute(@ByVal opencv_core.UMat uMat, @StdVector FloatBuffer floatBuffer, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @ByRef(nullValue = "std::vector<cv::Point>()") @Const opencv_core.PointVector pointVector);

        public native void compute(@ByVal opencv_core.UMat uMat, @StdVector FloatPointer floatPointer);

        public native void compute(@ByVal opencv_core.UMat uMat, @StdVector FloatPointer floatPointer, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @ByRef(nullValue = "std::vector<cv::Point>()") @Const opencv_core.PointVector pointVector);

        public native void compute(@ByVal opencv_core.UMat uMat, @StdVector float[] fArr);

        public native void compute(@ByVal opencv_core.UMat uMat, @StdVector float[] fArr, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @ByRef(nullValue = "std::vector<cv::Point>()") @Const opencv_core.PointVector pointVector);

        public native void computeGradient(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

        public native void computeGradient(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2);

        public native void computeGradient(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

        public native void computeGradient(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2);

        public native void computeGradient(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

        public native void computeGradient(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2);

        public native void copyTo(@ByRef HOGDescriptor hOGDescriptor);

        public native int derivAperture();

        public native HOGDescriptor derivAperture(int i);

        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.PointVector pointVector);

        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.PointVector pointVector, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @ByRef(nullValue = "std::vector<cv::Point>()") @Const opencv_core.PointVector pointVector2);

        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.PointVector pointVector, @StdVector DoubleBuffer doubleBuffer);

        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.PointVector pointVector, @StdVector DoubleBuffer doubleBuffer, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @ByRef(nullValue = "std::vector<cv::Point>()") @Const opencv_core.PointVector pointVector2);

        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.PointVector pointVector, @StdVector DoublePointer doublePointer);

        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.PointVector pointVector, @StdVector DoublePointer doublePointer, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @ByRef(nullValue = "std::vector<cv::Point>()") @Const opencv_core.PointVector pointVector2);

        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.PointVector pointVector, @StdVector double[] dArr);

        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.PointVector pointVector, @StdVector double[] dArr, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @ByRef(nullValue = "std::vector<cv::Point>()") @Const opencv_core.PointVector pointVector2);

        public native void detect(@ByVal opencv_core.Mat mat, @ByRef opencv_core.PointVector pointVector);

        public native void detect(@ByVal opencv_core.Mat mat, @ByRef opencv_core.PointVector pointVector, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @ByRef(nullValue = "std::vector<cv::Point>()") @Const opencv_core.PointVector pointVector2);

        public native void detect(@ByVal opencv_core.Mat mat, @ByRef opencv_core.PointVector pointVector, @StdVector DoubleBuffer doubleBuffer);

        public native void detect(@ByVal opencv_core.Mat mat, @ByRef opencv_core.PointVector pointVector, @StdVector DoubleBuffer doubleBuffer, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @ByRef(nullValue = "std::vector<cv::Point>()") @Const opencv_core.PointVector pointVector2);

        public native void detect(@ByVal opencv_core.Mat mat, @ByRef opencv_core.PointVector pointVector, @StdVector DoublePointer doublePointer);

        public native void detect(@ByVal opencv_core.Mat mat, @ByRef opencv_core.PointVector pointVector, @StdVector DoublePointer doublePointer, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @ByRef(nullValue = "std::vector<cv::Point>()") @Const opencv_core.PointVector pointVector2);

        public native void detect(@ByVal opencv_core.Mat mat, @ByRef opencv_core.PointVector pointVector, @StdVector double[] dArr);

        public native void detect(@ByVal opencv_core.Mat mat, @ByRef opencv_core.PointVector pointVector, @StdVector double[] dArr, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @ByRef(nullValue = "std::vector<cv::Point>()") @Const opencv_core.PointVector pointVector2);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.PointVector pointVector);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.PointVector pointVector, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @ByRef(nullValue = "std::vector<cv::Point>()") @Const opencv_core.PointVector pointVector2);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.PointVector pointVector, @StdVector DoubleBuffer doubleBuffer);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.PointVector pointVector, @StdVector DoubleBuffer doubleBuffer, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @ByRef(nullValue = "std::vector<cv::Point>()") @Const opencv_core.PointVector pointVector2);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.PointVector pointVector, @StdVector DoublePointer doublePointer);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.PointVector pointVector, @StdVector DoublePointer doublePointer, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @ByRef(nullValue = "std::vector<cv::Point>()") @Const opencv_core.PointVector pointVector2);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.PointVector pointVector, @StdVector double[] dArr);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.PointVector pointVector, @StdVector double[] dArr, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, @ByRef(nullValue = "std::vector<cv::Point>()") @Const opencv_core.PointVector pointVector2);

        public native void detectMultiScale(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector);

        public native void detectMultiScale(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, double d2, double d3, @Cast({"bool"}) boolean z);

        public native void detectMultiScale(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector DoubleBuffer doubleBuffer);

        public native void detectMultiScale(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector DoubleBuffer doubleBuffer, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, double d2, double d3, @Cast({"bool"}) boolean z);

        public native void detectMultiScale(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector DoublePointer doublePointer);

        public native void detectMultiScale(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector DoublePointer doublePointer, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, double d2, double d3, @Cast({"bool"}) boolean z);

        public native void detectMultiScale(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector double[] dArr);

        public native void detectMultiScale(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector double[] dArr, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, double d2, double d3, @Cast({"bool"}) boolean z);

        public native void detectMultiScale(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector);

        public native void detectMultiScale(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, double d2, double d3, @Cast({"bool"}) boolean z);

        public native void detectMultiScale(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector DoubleBuffer doubleBuffer);

        public native void detectMultiScale(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector DoubleBuffer doubleBuffer, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, double d2, double d3, @Cast({"bool"}) boolean z);

        public native void detectMultiScale(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector DoublePointer doublePointer);

        public native void detectMultiScale(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector DoublePointer doublePointer, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, double d2, double d3, @Cast({"bool"}) boolean z);

        public native void detectMultiScale(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector double[] dArr);

        public native void detectMultiScale(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector double[] dArr, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, double d2, double d3, @Cast({"bool"}) boolean z);

        public native void detectMultiScale(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector);

        public native void detectMultiScale(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, double d2, double d3, @Cast({"bool"}) boolean z);

        public native void detectMultiScale(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector DoubleBuffer doubleBuffer);

        public native void detectMultiScale(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector DoubleBuffer doubleBuffer, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, double d2, double d3, @Cast({"bool"}) boolean z);

        public native void detectMultiScale(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector DoublePointer doublePointer);

        public native void detectMultiScale(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector DoublePointer doublePointer, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, double d2, double d3, @Cast({"bool"}) boolean z);

        public native void detectMultiScale(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector double[] dArr);

        public native void detectMultiScale(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector double[] dArr, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, double d2, double d3, @Cast({"bool"}) boolean z);

        public native void detectMultiScaleROI(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector DetectionROI detectionROI);

        public native void detectMultiScaleROI(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector DetectionROI detectionROI, double d, int i);

        public native void detectMultiScaleROI(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector DetectionROI detectionROI);

        public native void detectMultiScaleROI(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector DetectionROI detectionROI, double d, int i);

        public native void detectMultiScaleROI(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector DetectionROI detectionROI);

        public native void detectMultiScaleROI(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector DetectionROI detectionROI, double d, int i);

        public native void detectROI(@ByVal opencv_core.GpuMat gpuMat, @ByRef @Const opencv_core.PointVector pointVector, @ByRef opencv_core.PointVector pointVector2, @StdVector DoubleBuffer doubleBuffer);

        public native void detectROI(@ByVal opencv_core.GpuMat gpuMat, @ByRef @Const opencv_core.PointVector pointVector, @ByRef opencv_core.PointVector pointVector2, @StdVector DoubleBuffer doubleBuffer, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2);

        public native void detectROI(@ByVal opencv_core.GpuMat gpuMat, @ByRef @Const opencv_core.PointVector pointVector, @ByRef opencv_core.PointVector pointVector2, @StdVector DoublePointer doublePointer);

        public native void detectROI(@ByVal opencv_core.GpuMat gpuMat, @ByRef @Const opencv_core.PointVector pointVector, @ByRef opencv_core.PointVector pointVector2, @StdVector DoublePointer doublePointer, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2);

        public native void detectROI(@ByVal opencv_core.GpuMat gpuMat, @ByRef @Const opencv_core.PointVector pointVector, @ByRef opencv_core.PointVector pointVector2, @StdVector double[] dArr);

        public native void detectROI(@ByVal opencv_core.GpuMat gpuMat, @ByRef @Const opencv_core.PointVector pointVector, @ByRef opencv_core.PointVector pointVector2, @StdVector double[] dArr, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2);

        public native void detectROI(@ByVal opencv_core.Mat mat, @ByRef @Const opencv_core.PointVector pointVector, @ByRef opencv_core.PointVector pointVector2, @StdVector DoubleBuffer doubleBuffer);

        public native void detectROI(@ByVal opencv_core.Mat mat, @ByRef @Const opencv_core.PointVector pointVector, @ByRef opencv_core.PointVector pointVector2, @StdVector DoubleBuffer doubleBuffer, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2);

        public native void detectROI(@ByVal opencv_core.Mat mat, @ByRef @Const opencv_core.PointVector pointVector, @ByRef opencv_core.PointVector pointVector2, @StdVector DoublePointer doublePointer);

        public native void detectROI(@ByVal opencv_core.Mat mat, @ByRef @Const opencv_core.PointVector pointVector, @ByRef opencv_core.PointVector pointVector2, @StdVector DoublePointer doublePointer, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2);

        public native void detectROI(@ByVal opencv_core.Mat mat, @ByRef @Const opencv_core.PointVector pointVector, @ByRef opencv_core.PointVector pointVector2, @StdVector double[] dArr);

        public native void detectROI(@ByVal opencv_core.Mat mat, @ByRef @Const opencv_core.PointVector pointVector, @ByRef opencv_core.PointVector pointVector2, @StdVector double[] dArr, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2);

        public native void detectROI(@ByVal opencv_core.UMat uMat, @ByRef @Const opencv_core.PointVector pointVector, @ByRef opencv_core.PointVector pointVector2, @StdVector DoubleBuffer doubleBuffer);

        public native void detectROI(@ByVal opencv_core.UMat uMat, @ByRef @Const opencv_core.PointVector pointVector, @ByRef opencv_core.PointVector pointVector2, @StdVector DoubleBuffer doubleBuffer, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2);

        public native void detectROI(@ByVal opencv_core.UMat uMat, @ByRef @Const opencv_core.PointVector pointVector, @ByRef opencv_core.PointVector pointVector2, @StdVector DoublePointer doublePointer);

        public native void detectROI(@ByVal opencv_core.UMat uMat, @ByRef @Const opencv_core.PointVector pointVector, @ByRef opencv_core.PointVector pointVector2, @StdVector DoublePointer doublePointer, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2);

        public native void detectROI(@ByVal opencv_core.UMat uMat, @ByRef @Const opencv_core.PointVector pointVector, @ByRef opencv_core.PointVector pointVector2, @StdVector double[] dArr);

        public native void detectROI(@ByVal opencv_core.UMat uMat, @ByRef @Const opencv_core.PointVector pointVector, @ByRef opencv_core.PointVector pointVector2, @StdVector double[] dArr, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2);

        public native float free_coef();

        public native HOGDescriptor free_coef(float f);

        public native HOGDescriptor gammaCorrection(boolean z);

        @Cast({"bool"})
        public native boolean gammaCorrection();

        @Cast({"size_t"})
        public native long getDescriptorSize();

        public native double getWinSigma();

        public native void groupRectangles(@ByRef opencv_core.RectVector rectVector, @StdVector DoubleBuffer doubleBuffer, int i, double d);

        public native void groupRectangles(@ByRef opencv_core.RectVector rectVector, @StdVector DoublePointer doublePointer, int i, double d);

        public native void groupRectangles(@ByRef opencv_core.RectVector rectVector, @StdVector double[] dArr, int i, double d);

        @Cast({"cv::HOGDescriptor::HistogramNormType"})
        public native int histogramNormType();

        public native HOGDescriptor histogramNormType(int i);

        @Cast({"bool"})
        public native boolean load(@opencv_core.Str String str);

        @Cast({"bool"})
        public native boolean load(@opencv_core.Str String str, @opencv_core.Str String str2);

        @Cast({"bool"})
        public native boolean load(@opencv_core.Str BytePointer bytePointer);

        @Cast({"bool"})
        public native boolean load(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

        public native int nbins();

        public native HOGDescriptor nbins(int i);

        public native int nlevels();

        public native HOGDescriptor nlevels(int i);

        @ByRef
        public native opencv_core.UMat oclSvmDetector();

        public native HOGDescriptor oclSvmDetector(opencv_core.UMat uMat);

        @Cast({"bool"})
        public native boolean read(@ByRef opencv_core.FileNode fileNode);

        public native void save(@opencv_core.Str String str);

        public native void save(@opencv_core.Str String str, @opencv_core.Str String str2);

        public native void save(@opencv_core.Str BytePointer bytePointer);

        public native void save(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

        public native void setSVMDetector(@ByVal opencv_core.GpuMat gpuMat);

        public native void setSVMDetector(@ByVal opencv_core.Mat mat);

        public native void setSVMDetector(@ByVal opencv_core.UMat uMat);

        public native HOGDescriptor signedGradient(boolean z);

        @Cast({"bool"})
        public native boolean signedGradient();

        @StdVector
        public native FloatPointer svmDetector();

        public native HOGDescriptor svmDetector(FloatPointer floatPointer);

        public native double winSigma();

        public native HOGDescriptor winSigma(double d);

        @ByRef
        public native opencv_core.Size winSize();

        public native HOGDescriptor winSize(opencv_core.Size size);

        public native void write(@ByRef opencv_core.FileStorage fileStorage, @opencv_core.Str String str);

        public native void write(@ByRef opencv_core.FileStorage fileStorage, @opencv_core.Str BytePointer bytePointer);

        static {
            Loader.load();
        }

        public HOGDescriptor(Pointer p) {
            super(p);
        }

        public HOGDescriptor(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public HOGDescriptor position(long position) {
            return (HOGDescriptor) super.position(position);
        }

        public HOGDescriptor() {
            super((Pointer) null);
            allocate();
        }

        public HOGDescriptor(@ByVal opencv_core.Size _winSize, @ByVal opencv_core.Size _blockSize, @ByVal opencv_core.Size _blockStride, @ByVal opencv_core.Size _cellSize, int _nbins, int _derivAperture, double _winSigma, @Cast({"cv::HOGDescriptor::HistogramNormType"}) int _histogramNormType, double _L2HysThreshold, @Cast({"bool"}) boolean _gammaCorrection, int _nlevels, @Cast({"bool"}) boolean _signedGradient) {
            super((Pointer) null);
            allocate(_winSize, _blockSize, _blockStride, _cellSize, _nbins, _derivAperture, _winSigma, _histogramNormType, _L2HysThreshold, _gammaCorrection, _nlevels, _signedGradient);
        }

        public HOGDescriptor(@ByVal opencv_core.Size _winSize, @ByVal opencv_core.Size _blockSize, @ByVal opencv_core.Size _blockStride, @ByVal opencv_core.Size _cellSize, int _nbins) {
            super((Pointer) null);
            allocate(_winSize, _blockSize, _blockStride, _cellSize, _nbins);
        }

        public HOGDescriptor(@opencv_core.Str BytePointer filename) {
            super((Pointer) null);
            allocate(filename);
        }

        public HOGDescriptor(@opencv_core.Str String filename) {
            super((Pointer) null);
            allocate(filename);
        }

        public HOGDescriptor(@ByRef @Const HOGDescriptor d) {
            super((Pointer) null);
            allocate(d);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class QRCodeDetector extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        @StdString
        public native String decode(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        @StdString
        public native String decode(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat3);

        @StdString
        public native BytePointer decode(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        @StdString
        public native BytePointer decode(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat3);

        @StdString
        public native BytePointer decode(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        @StdString
        public native BytePointer decode(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat3);

        @Cast({"bool"})
        public native boolean detect(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        @Cast({"bool"})
        public native boolean detect(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        @Cast({"bool"})
        public native boolean detect(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        @StdString
        public native String detectAndDecode(@ByVal opencv_core.UMat uMat);

        @StdString
        public native String detectAndDecode(@ByVal opencv_core.UMat uMat, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat2, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat3);

        @StdString
        public native BytePointer detectAndDecode(@ByVal opencv_core.GpuMat gpuMat);

        @StdString
        public native BytePointer detectAndDecode(@ByVal opencv_core.GpuMat gpuMat, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat2, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat3);

        @StdString
        public native BytePointer detectAndDecode(@ByVal opencv_core.Mat mat);

        @StdString
        public native BytePointer detectAndDecode(@ByVal opencv_core.Mat mat, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat2, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat3);

        public native void setEpsX(double d);

        public native void setEpsY(double d);

        static {
            Loader.load();
        }

        public QRCodeDetector(Pointer p) {
            super(p);
        }

        public QRCodeDetector(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public QRCodeDetector position(long position) {
            return (QRCodeDetector) super.position(position);
        }

        public QRCodeDetector() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class DetectionBasedTracker extends Pointer {
        public static final int DETECTED = 1;
        public static final int DETECTED_NOT_SHOWN_YET = 0;
        public static final int DETECTED_TEMPORARY_LOST = 2;
        public static final int WRONG_OBJECT = 3;

        private native void allocate(@opencv_core.Ptr IDetector iDetector, @opencv_core.Ptr IDetector iDetector2, @ByRef @Const Parameters parameters);

        public native int addObject(@ByRef @Const opencv_core.Rect rect);

        public native void getObjects(@Cast({"cv::DetectionBasedTracker::Object*"}) @StdVector opencv_core.IntIntPair intIntPair);

        public native void getObjects(@ByRef opencv_core.RectVector rectVector);

        public native void getObjects(@StdVector ExtObject extObject);

        @ByRef
        @Const
        public native Parameters getParameters();

        public native void process(@ByRef @Const opencv_core.Mat mat);

        public native void resetTracking();

        @Cast({"bool"})
        public native boolean run();

        @Cast({"bool"})
        public native boolean setParameters(@ByRef @Const Parameters parameters);

        public native void stop();

        static {
            Loader.load();
        }

        public DetectionBasedTracker(Pointer p) {
            super(p);
        }

        @NoOffset
        public static class Parameters extends Pointer {
            private native void allocate();

            private native void allocateArray(long j);

            public native int maxTrackLifetime();

            public native Parameters maxTrackLifetime(int i);

            public native int minDetectionPeriod();

            public native Parameters minDetectionPeriod(int i);

            static {
                Loader.load();
            }

            public Parameters(Pointer p) {
                super(p);
            }

            public Parameters(long size) {
                super((Pointer) null);
                allocateArray(size);
            }

            public Parameters position(long position) {
                return (Parameters) super.position(position);
            }

            public Parameters() {
                super((Pointer) null);
                allocate();
            }
        }

        @NoOffset
        public static class IDetector extends Pointer {
            public native void detect(@ByRef @Const opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector);

            @ByVal
            public native opencv_core.Size getMaxObjectSize();

            public native int getMinNeighbours();

            @ByVal
            public native opencv_core.Size getMinObjectSize();

            public native float getScaleFactor();

            public native void setMaxObjectSize(@ByRef @Const opencv_core.Size size);

            public native void setMinNeighbours(int i);

            public native void setMinObjectSize(@ByRef @Const opencv_core.Size size);

            public native void setScaleFactor(float f);

            static {
                Loader.load();
            }

            public IDetector(Pointer p) {
                super(p);
            }
        }

        public DetectionBasedTracker(@opencv_core.Ptr IDetector mainDetector, @opencv_core.Ptr IDetector trackingDetector, @ByRef @Const Parameters params) {
            super((Pointer) null);
            allocate(mainDetector, trackingDetector, params);
        }

        @NoOffset
        public static class ExtObject extends Pointer {
            private native void allocate(int i, @ByVal opencv_core.Rect rect, @Cast({"cv::DetectionBasedTracker::ObjectStatus"}) int i2);

            public native int id();

            public native ExtObject id(int i);

            @ByRef
            public native opencv_core.Rect location();

            public native ExtObject location(opencv_core.Rect rect);

            @Cast({"cv::DetectionBasedTracker::ObjectStatus"})
            public native int status();

            public native ExtObject status(int i);

            static {
                Loader.load();
            }

            public ExtObject(Pointer p) {
                super(p);
            }

            public ExtObject(int _id, @ByVal opencv_core.Rect _location, @Cast({"cv::DetectionBasedTracker::ObjectStatus"}) int _status) {
                super((Pointer) null);
                allocate(_id, _location, _status);
            }
        }
    }
}
