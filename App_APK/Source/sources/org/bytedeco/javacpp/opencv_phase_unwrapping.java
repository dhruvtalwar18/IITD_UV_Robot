package org.bytedeco.javacpp;

import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Const;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.annotation.NoOffset;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.presets.opencv_core;

public class opencv_phase_unwrapping extends org.bytedeco.javacpp.presets.opencv_phase_unwrapping {
    static {
        Loader.load();
    }

    @Namespace("cv::phase_unwrapping")
    public static class PhaseUnwrapping extends opencv_core.Algorithm {
        public native void unwrapPhaseMap(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void unwrapPhaseMap(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat3);

        public native void unwrapPhaseMap(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void unwrapPhaseMap(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat3);

        public native void unwrapPhaseMap(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void unwrapPhaseMap(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat3);

        static {
            Loader.load();
        }

        public PhaseUnwrapping(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::phase_unwrapping")
    public static class HistogramPhaseUnwrapping extends PhaseUnwrapping {
        @opencv_core.Ptr
        public static native HistogramPhaseUnwrapping create();

        @opencv_core.Ptr
        public static native HistogramPhaseUnwrapping create(@ByRef(nullValue = "cv::phase_unwrapping::HistogramPhaseUnwrapping::Params()") @Const Params params);

        public native void getInverseReliabilityMap(@ByVal opencv_core.GpuMat gpuMat);

        public native void getInverseReliabilityMap(@ByVal opencv_core.Mat mat);

        public native void getInverseReliabilityMap(@ByVal opencv_core.UMat uMat);

        static {
            Loader.load();
        }

        public HistogramPhaseUnwrapping(Pointer p) {
            super(p);
        }

        @NoOffset
        public static class Params extends Pointer {
            private native void allocate();

            private native void allocateArray(long j);

            public native int height();

            public native Params height(int i);

            public native float histThresh();

            public native Params histThresh(float f);

            public native int nbrOfLargeBins();

            public native Params nbrOfLargeBins(int i);

            public native int nbrOfSmallBins();

            public native Params nbrOfSmallBins(int i);

            public native int width();

            public native Params width(int i);

            static {
                Loader.load();
            }

            public Params(Pointer p) {
                super(p);
            }

            public Params(long size) {
                super((Pointer) null);
                allocateArray(size);
            }

            public Params position(long position) {
                return (Params) super.position(position);
            }

            public Params() {
                super((Pointer) null);
                allocate();
            }
        }
    }
}
