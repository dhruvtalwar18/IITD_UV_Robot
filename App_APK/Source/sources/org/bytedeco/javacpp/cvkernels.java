package org.bytedeco.javacpp;

import java.nio.DoubleBuffer;
import java.util.Arrays;
import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.MemberSetter;
import org.bytedeco.javacpp.annotation.Name;
import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.opencv_core;

@Properties(inherit = {opencv_core.class}, value = {@Platform(compiler = {"fastfpu"}, define = {"MAX_SIZE 16", "CV_INLINE static inline"}, include = {"cvkernels.h"})})
public class cvkernels {
    public static native void multiWarpColorTransform32F(KernelData kernelData, int i, opencv_core.CvRect cvRect, opencv_core.CvScalar cvScalar);

    public static native void multiWarpColorTransform8U(KernelData kernelData, int i, opencv_core.CvRect cvRect, opencv_core.CvScalar cvScalar);

    static {
        Loader.load();
    }

    public static class KernelData extends Pointer {
        private DoubleBuffer[] dstDstDotBuffers = new DoubleBuffer[1];

        private native void allocate();

        private native void allocateArray(long j);

        @ByRef
        @Name({"operator="})
        private native KernelData put(@ByRef KernelData kernelData);

        @MemberSetter
        @Name({"dstDstDot"})
        private native KernelData setDstDstDot(DoubleBuffer doubleBuffer);

        public native KernelData H1(opencv_core.CvMat cvMat);

        public native opencv_core.CvMat H1();

        public native KernelData H2(opencv_core.CvMat cvMat);

        public native opencv_core.CvMat H2();

        public native KernelData X(opencv_core.CvMat cvMat);

        public native opencv_core.CvMat X();

        public native int dstCount();

        public native KernelData dstCount(int i);

        public native int dstCountOutlier();

        public native KernelData dstCountOutlier(int i);

        public native int dstCountZero();

        public native KernelData dstCountZero(int i);

        public native KernelData dstImg(opencv_core.IplImage iplImage);

        public native opencv_core.IplImage dstImg();

        public native KernelData mask(opencv_core.IplImage iplImage);

        public native opencv_core.IplImage mask();

        public native double outlierThreshold();

        public native KernelData outlierThreshold(double d);

        public native KernelData srcDotImg(opencv_core.IplImage iplImage);

        public native opencv_core.IplImage srcDotImg();

        public native double srcDstDot();

        public native KernelData srcDstDot(double d);

        public native KernelData srcImg(opencv_core.IplImage iplImage);

        public native opencv_core.IplImage srcImg();

        public native KernelData srcImg2(opencv_core.IplImage iplImage);

        public native opencv_core.IplImage srcImg2();

        public native KernelData subImg(opencv_core.IplImage iplImage);

        public native opencv_core.IplImage subImg();

        public native KernelData transImg(opencv_core.IplImage iplImage);

        public native opencv_core.IplImage transImg();

        public native double zeroThreshold();

        public native KernelData zeroThreshold(double d);

        static {
            Loader.load();
        }

        public KernelData() {
            allocate();
        }

        public KernelData(long size) {
            allocateArray(size);
        }

        public KernelData(Pointer p) {
            super(p);
        }

        public KernelData position(long position) {
            return (KernelData) super.position(position);
        }

        public DoubleBuffer dstDstDot() {
            return this.dstDstDotBuffers[(int) this.position];
        }

        public KernelData dstDstDot(DoubleBuffer dstDstDot) {
            if (((long) this.dstDstDotBuffers.length) < this.capacity) {
                this.dstDstDotBuffers = (DoubleBuffer[]) Arrays.copyOf(this.dstDstDotBuffers, (int) this.capacity);
            }
            this.dstDstDotBuffers[(int) this.position] = dstDstDot;
            return setDstDstDot(dstDstDot);
        }
    }
}
