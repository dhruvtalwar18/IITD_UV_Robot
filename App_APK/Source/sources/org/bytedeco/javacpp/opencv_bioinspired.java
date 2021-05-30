package org.bytedeco.javacpp;

import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Const;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.annotation.NoOffset;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.presets.opencv_core;

public class opencv_bioinspired extends org.bytedeco.javacpp.presets.opencv_bioinspired {
    public static final int RETINA_COLOR_BAYER = 2;
    public static final int RETINA_COLOR_DIAGONAL = 1;
    public static final int RETINA_COLOR_RANDOM = 0;

    static {
        Loader.load();
    }

    @Namespace("cv::bioinspired")
    public static class RetinaParameters extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        @ByRef
        public native IplMagnoParameters IplMagno();

        public native RetinaParameters IplMagno(IplMagnoParameters iplMagnoParameters);

        @ByRef
        public native OPLandIplParvoParameters OPLandIplParvo();

        public native RetinaParameters OPLandIplParvo(OPLandIplParvoParameters oPLandIplParvoParameters);

        static {
            Loader.load();
        }

        public RetinaParameters() {
            super((Pointer) null);
            allocate();
        }

        public RetinaParameters(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public RetinaParameters(Pointer p) {
            super(p);
        }

        public RetinaParameters position(long position) {
            return (RetinaParameters) super.position(position);
        }

        @NoOffset
        public static class OPLandIplParvoParameters extends Pointer {
            private native void allocate();

            private native void allocateArray(long j);

            public native OPLandIplParvoParameters colorMode(boolean z);

            @Cast({"bool"})
            public native boolean colorMode();

            public native float ganglionCellsSensitivity();

            public native OPLandIplParvoParameters ganglionCellsSensitivity(float f);

            public native float hcellsSpatialConstant();

            public native OPLandIplParvoParameters hcellsSpatialConstant(float f);

            public native float hcellsTemporalConstant();

            public native OPLandIplParvoParameters hcellsTemporalConstant(float f);

            public native float horizontalCellsGain();

            public native OPLandIplParvoParameters horizontalCellsGain(float f);

            public native OPLandIplParvoParameters normaliseOutput(boolean z);

            @Cast({"bool"})
            public native boolean normaliseOutput();

            public native float photoreceptorsLocalAdaptationSensitivity();

            public native OPLandIplParvoParameters photoreceptorsLocalAdaptationSensitivity(float f);

            public native float photoreceptorsSpatialConstant();

            public native OPLandIplParvoParameters photoreceptorsSpatialConstant(float f);

            public native float photoreceptorsTemporalConstant();

            public native OPLandIplParvoParameters photoreceptorsTemporalConstant(float f);

            static {
                Loader.load();
            }

            public OPLandIplParvoParameters(Pointer p) {
                super(p);
            }

            public OPLandIplParvoParameters(long size) {
                super((Pointer) null);
                allocateArray(size);
            }

            public OPLandIplParvoParameters position(long position) {
                return (OPLandIplParvoParameters) super.position(position);
            }

            public OPLandIplParvoParameters() {
                super((Pointer) null);
                allocate();
            }
        }

        @NoOffset
        public static class IplMagnoParameters extends Pointer {
            private native void allocate();

            private native void allocateArray(long j);

            public native float V0CompressionParameter();

            public native IplMagnoParameters V0CompressionParameter(float f);

            public native float amacrinCellsTemporalCutFrequency();

            public native IplMagnoParameters amacrinCellsTemporalCutFrequency(float f);

            public native float localAdaptintegration_k();

            public native IplMagnoParameters localAdaptintegration_k(float f);

            public native float localAdaptintegration_tau();

            public native IplMagnoParameters localAdaptintegration_tau(float f);

            public native IplMagnoParameters normaliseOutput(boolean z);

            @Cast({"bool"})
            public native boolean normaliseOutput();

            public native float parasolCells_beta();

            public native IplMagnoParameters parasolCells_beta(float f);

            public native float parasolCells_k();

            public native IplMagnoParameters parasolCells_k(float f);

            public native float parasolCells_tau();

            public native IplMagnoParameters parasolCells_tau(float f);

            static {
                Loader.load();
            }

            public IplMagnoParameters(Pointer p) {
                super(p);
            }

            public IplMagnoParameters(long size) {
                super((Pointer) null);
                allocateArray(size);
            }

            public IplMagnoParameters position(long position) {
                return (IplMagnoParameters) super.position(position);
            }

            public IplMagnoParameters() {
                super((Pointer) null);
                allocate();
            }
        }
    }

    @Namespace("cv::bioinspired")
    public static class Retina extends opencv_core.Algorithm {
        @opencv_core.Ptr
        public static native Retina create(@ByVal opencv_core.Size size);

        @opencv_core.Ptr
        public static native Retina create(@ByVal opencv_core.Size size, @Cast({"const bool"}) boolean z);

        @opencv_core.Ptr
        public static native Retina create(@ByVal opencv_core.Size size, @Cast({"const bool"}) boolean z, int i, @Cast({"const bool"}) boolean z2, float f, float f2);

        public native void activateContoursProcessing(@Cast({"const bool"}) boolean z);

        public native void activateMovingContoursProcessing(@Cast({"const bool"}) boolean z);

        public native void applyFastToneMapping(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void applyFastToneMapping(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void applyFastToneMapping(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void clearBuffers();

        @ByVal
        public native opencv_core.Size getInputSize();

        public native void getMagno(@ByVal opencv_core.GpuMat gpuMat);

        public native void getMagno(@ByVal opencv_core.Mat mat);

        public native void getMagno(@ByVal opencv_core.UMat uMat);

        @Const
        @ByVal
        public native opencv_core.Mat getMagnoRAW();

        public native void getMagnoRAW(@ByVal opencv_core.GpuMat gpuMat);

        public native void getMagnoRAW(@ByVal opencv_core.Mat mat);

        public native void getMagnoRAW(@ByVal opencv_core.UMat uMat);

        @ByVal
        public native opencv_core.Size getOutputSize();

        @ByVal
        public native RetinaParameters getParameters();

        public native void getParvo(@ByVal opencv_core.GpuMat gpuMat);

        public native void getParvo(@ByVal opencv_core.Mat mat);

        public native void getParvo(@ByVal opencv_core.UMat uMat);

        @Const
        @ByVal
        public native opencv_core.Mat getParvoRAW();

        public native void getParvoRAW(@ByVal opencv_core.GpuMat gpuMat);

        public native void getParvoRAW(@ByVal opencv_core.Mat mat);

        public native void getParvoRAW(@ByVal opencv_core.UMat uMat);

        @opencv_core.Str
        public native BytePointer printSetup();

        public native void run(@ByVal opencv_core.GpuMat gpuMat);

        public native void run(@ByVal opencv_core.Mat mat);

        public native void run(@ByVal opencv_core.UMat uMat);

        public native void setColorSaturation();

        public native void setColorSaturation(@Cast({"const bool"}) boolean z, float f);

        public native void setup();

        public native void setup(@opencv_core.Str String str, @Cast({"const bool"}) boolean z);

        public native void setup(@opencv_core.Str BytePointer bytePointer, @Cast({"const bool"}) boolean z);

        public native void setup(@ByVal RetinaParameters retinaParameters);

        public native void setup(@ByRef opencv_core.FileStorage fileStorage);

        public native void setup(@ByRef opencv_core.FileStorage fileStorage, @Cast({"const bool"}) boolean z);

        public native void setupIPLMagnoChannel();

        public native void setupIPLMagnoChannel(@Cast({"const bool"}) boolean z, float f, float f2, float f3, float f4, float f5, float f6, float f7);

        public native void setupOPLandIPLParvoChannel();

        public native void setupOPLandIPLParvoChannel(@Cast({"const bool"}) boolean z, @Cast({"const bool"}) boolean z2, float f, float f2, float f3, float f4, float f5, float f6, float f7);

        public native void write(@opencv_core.Str String str);

        public native void write(@opencv_core.Str BytePointer bytePointer);

        public native void write(@ByRef opencv_core.FileStorage fileStorage);

        static {
            Loader.load();
        }

        public Retina(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::bioinspired")
    public static class RetinaFastToneMapping extends opencv_core.Algorithm {
        @opencv_core.Ptr
        public static native RetinaFastToneMapping create(@ByVal opencv_core.Size size);

        public native void applyFastToneMapping(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void applyFastToneMapping(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void applyFastToneMapping(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void setup();

        public native void setup(float f, float f2, float f3);

        static {
            Loader.load();
        }

        public RetinaFastToneMapping(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::bioinspired")
    @NoOffset
    public static class SegmentationParameters extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        public native float contextEnergy_spatialConstant();

        public native SegmentationParameters contextEnergy_spatialConstant(float f);

        public native float contextEnergy_temporalConstant();

        public native SegmentationParameters contextEnergy_temporalConstant(float f);

        public native float localEnergy_spatialConstant();

        public native SegmentationParameters localEnergy_spatialConstant(float f);

        public native float localEnergy_temporalConstant();

        public native SegmentationParameters localEnergy_temporalConstant(float f);

        public native float neighborhoodEnergy_spatialConstant();

        public native SegmentationParameters neighborhoodEnergy_spatialConstant(float f);

        public native float neighborhoodEnergy_temporalConstant();

        public native SegmentationParameters neighborhoodEnergy_temporalConstant(float f);

        public native float thresholdOFF();

        public native SegmentationParameters thresholdOFF(float f);

        public native float thresholdON();

        public native SegmentationParameters thresholdON(float f);

        static {
            Loader.load();
        }

        public SegmentationParameters(Pointer p) {
            super(p);
        }

        public SegmentationParameters(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public SegmentationParameters position(long position) {
            return (SegmentationParameters) super.position(position);
        }

        public SegmentationParameters() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::bioinspired")
    public static class TransientAreasSegmentationModule extends opencv_core.Algorithm {
        @opencv_core.Ptr
        public static native TransientAreasSegmentationModule create(@ByVal opencv_core.Size size);

        public native void clearAllBuffers();

        @ByVal
        public native SegmentationParameters getParameters();

        public native void getSegmentationPicture(@ByVal opencv_core.GpuMat gpuMat);

        public native void getSegmentationPicture(@ByVal opencv_core.Mat mat);

        public native void getSegmentationPicture(@ByVal opencv_core.UMat uMat);

        @ByVal
        public native opencv_core.Size getSize();

        @opencv_core.Str
        public native BytePointer printSetup();

        public native void run(@ByVal opencv_core.GpuMat gpuMat);

        public native void run(@ByVal opencv_core.GpuMat gpuMat, int i);

        public native void run(@ByVal opencv_core.Mat mat);

        public native void run(@ByVal opencv_core.Mat mat, int i);

        public native void run(@ByVal opencv_core.UMat uMat);

        public native void run(@ByVal opencv_core.UMat uMat, int i);

        public native void setup();

        public native void setup(@opencv_core.Str String str, @Cast({"const bool"}) boolean z);

        public native void setup(@opencv_core.Str BytePointer bytePointer, @Cast({"const bool"}) boolean z);

        public native void setup(@ByVal SegmentationParameters segmentationParameters);

        public native void setup(@ByRef opencv_core.FileStorage fileStorage);

        public native void setup(@ByRef opencv_core.FileStorage fileStorage, @Cast({"const bool"}) boolean z);

        public native void write(@opencv_core.Str String str);

        public native void write(@opencv_core.Str BytePointer bytePointer);

        public native void write(@ByRef opencv_core.FileStorage fileStorage);

        static {
            Loader.load();
        }

        public TransientAreasSegmentationModule(Pointer p) {
            super(p);
        }
    }
}
