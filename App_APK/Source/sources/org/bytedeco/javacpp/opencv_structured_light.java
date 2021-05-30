package org.bytedeco.javacpp;

import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Const;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.annotation.NoOffset;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.presets.opencv_core;

public class opencv_structured_light extends org.bytedeco.javacpp.presets.opencv_structured_light {
    public static final int DECODE_3D_UNDERWORLD = 0;
    public static final int FAPS = 2;
    public static final int FTP = 0;
    public static final int PSP = 1;

    static {
        Loader.load();
    }

    @Namespace("cv::structured_light")
    public static class StructuredLightPattern extends opencv_core.Algorithm {
        @Cast({"bool"})
        public native boolean decode(@ByRef @Const opencv_core.MatVectorVector matVectorVector, @ByVal opencv_core.GpuMat gpuMat);

        @Cast({"bool"})
        public native boolean decode(@ByRef @Const opencv_core.MatVectorVector matVectorVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector2, int i);

        @Cast({"bool"})
        public native boolean decode(@ByRef @Const opencv_core.MatVectorVector matVectorVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector2, int i);

        @Cast({"bool"})
        public native boolean decode(@ByRef @Const opencv_core.MatVectorVector matVectorVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector2, int i);

        @Cast({"bool"})
        public native boolean decode(@ByRef @Const opencv_core.MatVectorVector matVectorVector, @ByVal opencv_core.Mat mat);

        @Cast({"bool"})
        public native boolean decode(@ByRef @Const opencv_core.MatVectorVector matVectorVector, @ByVal opencv_core.Mat mat, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector2, int i);

        @Cast({"bool"})
        public native boolean decode(@ByRef @Const opencv_core.MatVectorVector matVectorVector, @ByVal opencv_core.Mat mat, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector2, int i);

        @Cast({"bool"})
        public native boolean decode(@ByRef @Const opencv_core.MatVectorVector matVectorVector, @ByVal opencv_core.Mat mat, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector2, int i);

        @Cast({"bool"})
        public native boolean decode(@ByRef @Const opencv_core.MatVectorVector matVectorVector, @ByVal opencv_core.UMat uMat);

        @Cast({"bool"})
        public native boolean decode(@ByRef @Const opencv_core.MatVectorVector matVectorVector, @ByVal opencv_core.UMat uMat, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector2, int i);

        @Cast({"bool"})
        public native boolean decode(@ByRef @Const opencv_core.MatVectorVector matVectorVector, @ByVal opencv_core.UMat uMat, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector2, int i);

        @Cast({"bool"})
        public native boolean decode(@ByRef @Const opencv_core.MatVectorVector matVectorVector, @ByVal opencv_core.UMat uMat, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector2, int i);

        @Cast({"bool"})
        public native boolean generate(@ByVal opencv_core.GpuMatVector gpuMatVector);

        @Cast({"bool"})
        public native boolean generate(@ByVal opencv_core.MatVector matVector);

        @Cast({"bool"})
        public native boolean generate(@ByVal opencv_core.UMatVector uMatVector);

        static {
            Loader.load();
        }

        public StructuredLightPattern(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::structured_light")
    public static class GrayCodePattern extends StructuredLightPattern {
        @opencv_core.Ptr
        public static native GrayCodePattern create();

        @opencv_core.Ptr
        public static native GrayCodePattern create(int i, int i2);

        @opencv_core.Ptr
        public static native GrayCodePattern create(@ByRef(nullValue = "cv::structured_light::GrayCodePattern::Params()") @Const Params params);

        public native void getImagesForShadowMasks(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void getImagesForShadowMasks(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void getImagesForShadowMasks(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        @Cast({"size_t"})
        public native long getNumberOfPatternImages();

        @Cast({"bool"})
        public native boolean getProjPixel(@ByVal opencv_core.GpuMatVector gpuMatVector, int i, int i2, @ByRef opencv_core.Point point);

        @Cast({"bool"})
        public native boolean getProjPixel(@ByVal opencv_core.MatVector matVector, int i, int i2, @ByRef opencv_core.Point point);

        @Cast({"bool"})
        public native boolean getProjPixel(@ByVal opencv_core.UMatVector uMatVector, int i, int i2, @ByRef opencv_core.Point point);

        public native void setBlackThreshold(@Cast({"size_t"}) long j);

        public native void setWhiteThreshold(@Cast({"size_t"}) long j);

        static {
            Loader.load();
        }

        public GrayCodePattern(Pointer p) {
            super(p);
        }

        @NoOffset
        public static class Params extends Pointer {
            private native void allocate();

            private native void allocateArray(long j);

            public native int height();

            public native Params height(int i);

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

    @Namespace("cv::structured_light")
    public static class SinusoidalPattern extends StructuredLightPattern {
        @opencv_core.Ptr
        public static native SinusoidalPattern create();

        @opencv_core.Ptr
        public static native SinusoidalPattern create(@opencv_core.Ptr Params params);

        public native void computeDataModulationTerm(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void computeDataModulationTerm(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void computeDataModulationTerm(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void computeDataModulationTerm(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void computeDataModulationTerm(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void computeDataModulationTerm(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void computeDataModulationTerm(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void computeDataModulationTerm(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void computeDataModulationTerm(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void computePhaseMap(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat);

        public native void computePhaseMap(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat3);

        public native void computePhaseMap(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat);

        public native void computePhaseMap(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat3);

        public native void computePhaseMap(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat);

        public native void computePhaseMap(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat3);

        public native void computePhaseMap(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat);

        public native void computePhaseMap(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat3);

        public native void computePhaseMap(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat);

        public native void computePhaseMap(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat3);

        public native void computePhaseMap(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat);

        public native void computePhaseMap(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat3);

        public native void computePhaseMap(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat);

        public native void computePhaseMap(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat3);

        public native void computePhaseMap(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat);

        public native void computePhaseMap(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat3);

        public native void computePhaseMap(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat);

        public native void computePhaseMap(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat3);

        public native void findProCamMatches(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMatVector gpuMatVector);

        public native void findProCamMatches(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.MatVector matVector);

        public native void findProCamMatches(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.UMatVector uMatVector);

        public native void findProCamMatches(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.GpuMatVector gpuMatVector);

        public native void findProCamMatches(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.MatVector matVector);

        public native void findProCamMatches(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.UMatVector uMatVector);

        public native void findProCamMatches(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.GpuMatVector gpuMatVector);

        public native void findProCamMatches(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.MatVector matVector);

        public native void findProCamMatches(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMatVector uMatVector);

        public native void unwrapPhaseMap(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.Size size);

        public native void unwrapPhaseMap(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.Size size, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat2);

        public native void unwrapPhaseMap(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Size size);

        public native void unwrapPhaseMap(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Size size, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat2);

        public native void unwrapPhaseMap(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.Size size);

        public native void unwrapPhaseMap(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.Size size, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat2);

        public native void unwrapPhaseMap(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.Size size);

        public native void unwrapPhaseMap(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.Size size, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat2);

        public native void unwrapPhaseMap(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Size size);

        public native void unwrapPhaseMap(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Size size, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat2);

        public native void unwrapPhaseMap(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.Size size);

        public native void unwrapPhaseMap(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.Size size, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat2);

        public native void unwrapPhaseMap(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.Size size);

        public native void unwrapPhaseMap(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.Size size, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat2);

        public native void unwrapPhaseMap(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Size size);

        public native void unwrapPhaseMap(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Size size, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat2);

        public native void unwrapPhaseMap(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.Size size);

        public native void unwrapPhaseMap(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.Size size, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat2);

        static {
            Loader.load();
        }

        public SinusoidalPattern(Pointer p) {
            super(p);
        }

        @NoOffset
        public static class Params extends Pointer {
            private native void allocate();

            private native void allocateArray(long j);

            public native int height();

            public native Params height(int i);

            public native Params horizontal(boolean z);

            @Cast({"bool"})
            public native boolean horizontal();

            @ByRef
            public native opencv_core.Point2fVector markersLocation();

            public native Params markersLocation(opencv_core.Point2fVector point2fVector);

            public native int methodId();

            public native Params methodId(int i);

            public native int nbrOfPeriods();

            public native Params nbrOfPeriods(int i);

            public native int nbrOfPixelsBetweenMarkers();

            public native Params nbrOfPixelsBetweenMarkers(int i);

            public native Params setMarkers(boolean z);

            @Cast({"bool"})
            public native boolean setMarkers();

            public native float shiftValue();

            public native Params shiftValue(float f);

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
