package org.bytedeco.javacpp;

import java.nio.DoubleBuffer;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Const;
import org.bytedeco.javacpp.annotation.MemberGetter;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.annotation.NoOffset;
import org.bytedeco.javacpp.annotation.StdVector;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.opencv_features2d;
import org.bytedeco.javacpp.presets.opencv_core;

public class opencv_xfeatures2d extends org.bytedeco.javacpp.presets.opencv_xfeatures2d {
    @Namespace("cv::xfeatures2d")
    public static native void FASTForPointSet(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.KeyPointVector keyPointVector, int i);

    @Namespace("cv::xfeatures2d")
    public static native void FASTForPointSet(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.KeyPointVector keyPointVector, int i, @Cast({"bool"}) boolean z, @Cast({"cv::FastFeatureDetector::DetectorType"}) int i2);

    @Namespace("cv::xfeatures2d")
    public static native void FASTForPointSet(@ByVal opencv_core.Mat mat, @ByRef opencv_core.KeyPointVector keyPointVector, int i);

    @Namespace("cv::xfeatures2d")
    public static native void FASTForPointSet(@ByVal opencv_core.Mat mat, @ByRef opencv_core.KeyPointVector keyPointVector, int i, @Cast({"bool"}) boolean z, @Cast({"cv::FastFeatureDetector::DetectorType"}) int i2);

    @Namespace("cv::xfeatures2d")
    public static native void FASTForPointSet(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.KeyPointVector keyPointVector, int i);

    @Namespace("cv::xfeatures2d")
    public static native void FASTForPointSet(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.KeyPointVector keyPointVector, int i, @Cast({"bool"}) boolean z, @Cast({"cv::FastFeatureDetector::DetectorType"}) int i2);

    @Namespace("cv::xfeatures2d")
    public static native void matchGMS(@ByRef @Const opencv_core.Size size, @ByRef @Const opencv_core.Size size2, @ByRef @Const opencv_core.KeyPointVector keyPointVector, @ByRef @Const opencv_core.KeyPointVector keyPointVector2, @ByRef @Const opencv_core.DMatchVector dMatchVector, @ByRef opencv_core.DMatchVector dMatchVector2);

    @Namespace("cv::xfeatures2d")
    public static native void matchGMS(@ByRef @Const opencv_core.Size size, @ByRef @Const opencv_core.Size size2, @ByRef @Const opencv_core.KeyPointVector keyPointVector, @ByRef @Const opencv_core.KeyPointVector keyPointVector2, @ByRef @Const opencv_core.DMatchVector dMatchVector, @ByRef opencv_core.DMatchVector dMatchVector2, @Cast({"const bool"}) boolean z, @Cast({"const bool"}) boolean z2, double d);

    static {
        Loader.load();
    }

    @Namespace("cv::xfeatures2d")
    @NoOffset
    public static class FREAK extends opencv_features2d.Feature2D {
        public static final int NB_ORIENPAIRS = NB_ORIENPAIRS();
        public static final int NB_PAIRS = NB_PAIRS();
        public static final int NB_SCALES = NB_SCALES();

        @MemberGetter
        public static native int NB_ORIENPAIRS();

        @MemberGetter
        public static native int NB_PAIRS();

        @MemberGetter
        public static native int NB_SCALES();

        private native void allocate();

        private native void allocateArray(long j);

        @opencv_core.Ptr
        public static native FREAK create();

        @opencv_core.Ptr
        public static native FREAK create(@Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2, float f, int i, @StdVector IntBuffer intBuffer);

        @opencv_core.Ptr
        public static native FREAK create(@Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2, float f, int i, @StdVector IntPointer intPointer);

        @opencv_core.Ptr
        public static native FREAK create(@Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2, float f, int i, @StdVector int[] iArr);

        static {
            Loader.load();
        }

        public FREAK() {
            super((Pointer) null);
            allocate();
        }

        public FREAK(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public FREAK(Pointer p) {
            super(p);
        }

        public FREAK position(long position) {
            return (FREAK) super.position(position);
        }
    }

    @Namespace("cv::xfeatures2d")
    public static class StarDetector extends opencv_features2d.Feature2D {
        private native void allocate();

        private native void allocateArray(long j);

        @opencv_core.Ptr
        public static native StarDetector create();

        @opencv_core.Ptr
        public static native StarDetector create(int i, int i2, int i3, int i4, int i5);

        static {
            Loader.load();
        }

        public StarDetector() {
            super((Pointer) null);
            allocate();
        }

        public StarDetector(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public StarDetector(Pointer p) {
            super(p);
        }

        public StarDetector position(long position) {
            return (StarDetector) super.position(position);
        }
    }

    @Namespace("cv::xfeatures2d")
    public static class BriefDescriptorExtractor extends opencv_features2d.Feature2D {
        private native void allocate();

        private native void allocateArray(long j);

        @opencv_core.Ptr
        public static native BriefDescriptorExtractor create();

        @opencv_core.Ptr
        public static native BriefDescriptorExtractor create(int i, @Cast({"bool"}) boolean z);

        static {
            Loader.load();
        }

        public BriefDescriptorExtractor() {
            super((Pointer) null);
            allocate();
        }

        public BriefDescriptorExtractor(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public BriefDescriptorExtractor(Pointer p) {
            super(p);
        }

        public BriefDescriptorExtractor position(long position) {
            return (BriefDescriptorExtractor) super.position(position);
        }
    }

    @Namespace("cv::xfeatures2d")
    public static class LUCID extends opencv_features2d.Feature2D {
        private native void allocate();

        private native void allocateArray(long j);

        @opencv_core.Ptr
        public static native LUCID create();

        @opencv_core.Ptr
        public static native LUCID create(int i, int i2);

        static {
            Loader.load();
        }

        public LUCID() {
            super((Pointer) null);
            allocate();
        }

        public LUCID(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public LUCID(Pointer p) {
            super(p);
        }

        public LUCID position(long position) {
            return (LUCID) super.position(position);
        }
    }

    @Namespace("cv::xfeatures2d")
    public static class LATCH extends opencv_features2d.Feature2D {
        private native void allocate();

        private native void allocateArray(long j);

        @opencv_core.Ptr
        public static native LATCH create();

        @opencv_core.Ptr
        public static native LATCH create(int i, @Cast({"bool"}) boolean z, int i2, double d);

        static {
            Loader.load();
        }

        public LATCH() {
            super((Pointer) null);
            allocate();
        }

        public LATCH(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public LATCH(Pointer p) {
            super(p);
        }

        public LATCH position(long position) {
            return (LATCH) super.position(position);
        }
    }

    @Namespace("cv::xfeatures2d")
    public static class DAISY extends opencv_features2d.Feature2D {
        public static final int NRM_FULL = 102;
        public static final int NRM_NONE = 100;
        public static final int NRM_PARTIAL = 101;
        public static final int NRM_SIFT = 103;

        @opencv_core.Ptr
        public static native DAISY create();

        @opencv_core.Ptr
        public static native DAISY create(float f, int i, int i2, int i3, @Cast({"cv::xfeatures2d::DAISY::NormalizationType"}) int i4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat, @Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2);

        @opencv_core.Ptr
        public static native DAISY create(float f, int i, int i2, int i3, @Cast({"cv::xfeatures2d::DAISY::NormalizationType"}) int i4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat, @Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2);

        @opencv_core.Ptr
        public static native DAISY create(float f, int i, int i2, int i3, @Cast({"cv::xfeatures2d::DAISY::NormalizationType"}) int i4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat, @Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2);

        public native void GetDescriptor(double d, double d2, int i, FloatBuffer floatBuffer);

        public native void GetDescriptor(double d, double d2, int i, FloatPointer floatPointer);

        public native void GetDescriptor(double d, double d2, int i, float[] fArr);

        @Cast({"bool"})
        public native boolean GetDescriptor(double d, double d2, int i, FloatBuffer floatBuffer, DoubleBuffer doubleBuffer);

        @Cast({"bool"})
        public native boolean GetDescriptor(double d, double d2, int i, FloatPointer floatPointer, DoublePointer doublePointer);

        @Cast({"bool"})
        public native boolean GetDescriptor(double d, double d2, int i, float[] fArr, double[] dArr);

        public native void GetUnnormalizedDescriptor(double d, double d2, int i, FloatBuffer floatBuffer);

        public native void GetUnnormalizedDescriptor(double d, double d2, int i, FloatPointer floatPointer);

        public native void GetUnnormalizedDescriptor(double d, double d2, int i, float[] fArr);

        @Cast({"bool"})
        public native boolean GetUnnormalizedDescriptor(double d, double d2, int i, FloatBuffer floatBuffer, DoubleBuffer doubleBuffer);

        @Cast({"bool"})
        public native boolean GetUnnormalizedDescriptor(double d, double d2, int i, FloatPointer floatPointer, DoublePointer doublePointer);

        @Cast({"bool"})
        public native boolean GetUnnormalizedDescriptor(double d, double d2, int i, float[] fArr, double[] dArr);

        public native void compute(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void compute(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.GpuMat gpuMat2);

        public native void compute(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.Rect rect, @ByVal opencv_core.GpuMat gpuMat2);

        public native void compute(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByRef opencv_core.KeyPointVectorVector keyPointVectorVector, @ByVal opencv_core.GpuMatVector gpuMatVector2);

        public native void compute(@ByVal opencv_core.Mat mat, @ByRef opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.Mat mat2);

        public native void compute(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void compute(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Rect rect, @ByVal opencv_core.Mat mat2);

        public native void compute(@ByVal opencv_core.MatVector matVector, @ByRef opencv_core.KeyPointVectorVector keyPointVectorVector, @ByVal opencv_core.MatVector matVector2);

        public native void compute(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.UMat uMat2);

        public native void compute(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.Rect rect, @ByVal opencv_core.UMat uMat2);

        public native void compute(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void compute(@ByVal opencv_core.UMatVector uMatVector, @ByRef opencv_core.KeyPointVectorVector keyPointVectorVector, @ByVal opencv_core.UMatVector uMatVector2);

        static {
            Loader.load();
        }

        public DAISY(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::xfeatures2d")
    public static class MSDDetector extends opencv_features2d.Feature2D {
        private native void allocate();

        private native void allocateArray(long j);

        @opencv_core.Ptr
        public static native MSDDetector create();

        @opencv_core.Ptr
        public static native MSDDetector create(int i, int i2, int i3, int i4, float f, int i5, float f2, int i6, @Cast({"bool"}) boolean z);

        static {
            Loader.load();
        }

        public MSDDetector() {
            super((Pointer) null);
            allocate();
        }

        public MSDDetector(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public MSDDetector(Pointer p) {
            super(p);
        }

        public MSDDetector position(long position) {
            return (MSDDetector) super.position(position);
        }
    }

    @Namespace("cv::xfeatures2d")
    public static class VGG extends opencv_features2d.Feature2D {
        public static final int VGG_120 = 100;
        public static final int VGG_48 = 103;
        public static final int VGG_64 = 102;
        public static final int VGG_80 = 101;

        @opencv_core.Ptr
        public static native VGG create();

        @opencv_core.Ptr
        public static native VGG create(int i, float f, @Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2, float f2, @Cast({"bool"}) boolean z3);

        public native float getScaleFactor();

        public native float getSigma();

        @Cast({"bool"})
        public native boolean getUseNormalizeDescriptor();

        @Cast({"bool"})
        public native boolean getUseNormalizeImage();

        @Cast({"bool"})
        public native boolean getUseScaleOrientation();

        public native void setScaleFactor(float f);

        public native void setSigma(float f);

        public native void setUseNormalizeDescriptor(@Cast({"const bool"}) boolean z);

        public native void setUseNormalizeImage(@Cast({"const bool"}) boolean z);

        public native void setUseScaleOrientation(@Cast({"const bool"}) boolean z);

        static {
            Loader.load();
        }

        public VGG(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::xfeatures2d")
    public static class BoostDesc extends opencv_features2d.Feature2D {
        public static final int BGM = 100;
        public static final int BGM_BILINEAR = 102;
        public static final int BGM_HARD = 101;
        public static final int BINBOOST_128 = 301;
        public static final int BINBOOST_256 = 302;
        public static final int BINBOOST_64 = 300;
        public static final int LBGM = 200;

        @opencv_core.Ptr
        public static native BoostDesc create();

        @opencv_core.Ptr
        public static native BoostDesc create(int i, @Cast({"bool"}) boolean z, float f);

        public native float getScaleFactor();

        @Cast({"bool"})
        public native boolean getUseScaleOrientation();

        public native void setScaleFactor(float f);

        public native void setUseScaleOrientation(@Cast({"const bool"}) boolean z);

        static {
            Loader.load();
        }

        public BoostDesc(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::xfeatures2d")
    public static class PCTSignatures extends opencv_core.Algorithm {
        public static final int GAUSSIAN = 1;
        public static final int HEURISTIC = 2;
        public static final int L0_25 = 0;
        public static final int L0_5 = 1;
        public static final int L1 = 2;
        public static final int L2 = 3;
        public static final int L2SQUARED = 4;
        public static final int L5 = 5;
        public static final int L_INFINITY = 6;
        public static final int MINUS = 0;
        public static final int NORMAL = 2;
        public static final int REGULAR = 1;
        public static final int UNIFORM = 0;

        @opencv_core.Ptr
        public static native PCTSignatures create();

        @opencv_core.Ptr
        public static native PCTSignatures create(int i, int i2, int i3);

        @opencv_core.Ptr
        public static native PCTSignatures create(@ByRef @Const opencv_core.Point2fVector point2fVector, int i);

        @opencv_core.Ptr
        public static native PCTSignatures create(@ByRef @Const opencv_core.Point2fVector point2fVector, @StdVector IntBuffer intBuffer);

        @opencv_core.Ptr
        public static native PCTSignatures create(@ByRef @Const opencv_core.Point2fVector point2fVector, @StdVector IntPointer intPointer);

        @opencv_core.Ptr
        public static native PCTSignatures create(@ByRef @Const opencv_core.Point2fVector point2fVector, @StdVector int[] iArr);

        public static native void drawSignature(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

        public static native void drawSignature(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, float f, int i);

        public static native void drawSignature(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

        public static native void drawSignature(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, float f, int i);

        public static native void drawSignature(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

        public static native void drawSignature(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, float f, int i);

        public static native void generateInitPoints(@ByRef opencv_core.Point2fVector point2fVector, int i, int i2);

        public native void computeSignature(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void computeSignature(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void computeSignature(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void computeSignatures(@ByRef @Const opencv_core.MatVector matVector, @ByRef opencv_core.MatVector matVector2);

        public native int getClusterMinSize();

        public native int getDistanceFunction();

        public native float getDropThreshold();

        public native int getGrayscaleBits();

        public native int getInitSeedCount();

        @StdVector
        public native IntPointer getInitSeedIndexes();

        public native int getIterationCount();

        public native float getJoiningDistance();

        public native int getMaxClustersCount();

        public native int getSampleCount();

        @ByVal
        public native opencv_core.Point2fVector getSamplingPoints();

        public native float getWeightA();

        public native float getWeightB();

        public native float getWeightContrast();

        public native float getWeightEntropy();

        public native float getWeightL();

        public native float getWeightX();

        public native float getWeightY();

        public native int getWindowRadius();

        public native void setClusterMinSize(int i);

        public native void setDistanceFunction(int i);

        public native void setDropThreshold(float f);

        public native void setGrayscaleBits(int i);

        public native void setInitSeedIndexes(@StdVector IntBuffer intBuffer);

        public native void setInitSeedIndexes(@StdVector IntPointer intPointer);

        public native void setInitSeedIndexes(@StdVector int[] iArr);

        public native void setIterationCount(int i);

        public native void setJoiningDistance(float f);

        public native void setMaxClustersCount(int i);

        public native void setSamplingPoints(@ByVal opencv_core.Point2fVector point2fVector);

        public native void setTranslation(int i, float f);

        public native void setTranslations(@StdVector FloatBuffer floatBuffer);

        public native void setTranslations(@StdVector FloatPointer floatPointer);

        public native void setTranslations(@StdVector float[] fArr);

        public native void setWeight(int i, float f);

        public native void setWeightA(float f);

        public native void setWeightB(float f);

        public native void setWeightContrast(float f);

        public native void setWeightEntropy(float f);

        public native void setWeightL(float f);

        public native void setWeightX(float f);

        public native void setWeightY(float f);

        public native void setWeights(@StdVector FloatBuffer floatBuffer);

        public native void setWeights(@StdVector FloatPointer floatPointer);

        public native void setWeights(@StdVector float[] fArr);

        public native void setWindowRadius(int i);

        static {
            Loader.load();
        }

        public PCTSignatures(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::xfeatures2d")
    public static class PCTSignaturesSQFD extends opencv_core.Algorithm {
        @opencv_core.Ptr
        public static native PCTSignaturesSQFD create();

        @opencv_core.Ptr
        public static native PCTSignaturesSQFD create(int i, int i2, float f);

        public native float computeQuadraticFormDistance(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native float computeQuadraticFormDistance(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native float computeQuadraticFormDistance(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void computeQuadraticFormDistances(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.MatVector matVector, @StdVector FloatBuffer floatBuffer);

        public native void computeQuadraticFormDistances(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.MatVector matVector, @StdVector FloatPointer floatPointer);

        public native void computeQuadraticFormDistances(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.MatVector matVector, @StdVector float[] fArr);

        static {
            Loader.load();
        }

        public PCTSignaturesSQFD(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::xfeatures2d")
    @NoOffset
    public static class Elliptic_KeyPoint extends opencv_core.KeyPoint {
        private native void allocate();

        private native void allocate(@ByVal opencv_core.Point2f point2f, float f, @ByVal opencv_core.Size size, float f2, float f3);

        private native void allocateArray(long j);

        @ByRef
        public native opencv_core.Size2f axes();

        public native Elliptic_KeyPoint axes(opencv_core.Size2f size2f);

        public native float si();

        public native Elliptic_KeyPoint si(float f);

        @ByRef
        @Cast({"cv::Matx23f*"})
        public native FloatPointer transf();

        public native Elliptic_KeyPoint transf(FloatPointer floatPointer);

        static {
            Loader.load();
        }

        public Elliptic_KeyPoint(Pointer p) {
            super(p);
        }

        public Elliptic_KeyPoint(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Elliptic_KeyPoint position(long position) {
            return (Elliptic_KeyPoint) super.position(position);
        }

        public Elliptic_KeyPoint() {
            super((Pointer) null);
            allocate();
        }

        public Elliptic_KeyPoint(@ByVal opencv_core.Point2f pt, float angle, @ByVal opencv_core.Size axes, float size, float si) {
            super((Pointer) null);
            allocate(pt, angle, axes, size, si);
        }
    }

    @Namespace("cv::xfeatures2d")
    public static class HarrisLaplaceFeatureDetector extends opencv_features2d.Feature2D {
        private native void allocate();

        private native void allocateArray(long j);

        @opencv_core.Ptr
        public static native HarrisLaplaceFeatureDetector create();

        @opencv_core.Ptr
        public static native HarrisLaplaceFeatureDetector create(int i, float f, float f2, int i2, int i3);

        static {
            Loader.load();
        }

        public HarrisLaplaceFeatureDetector() {
            super((Pointer) null);
            allocate();
        }

        public HarrisLaplaceFeatureDetector(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public HarrisLaplaceFeatureDetector(Pointer p) {
            super(p);
        }

        public HarrisLaplaceFeatureDetector position(long position) {
            return (HarrisLaplaceFeatureDetector) super.position(position);
        }
    }

    @Namespace("cv::xfeatures2d")
    public static class AffineFeature2D extends opencv_features2d.Feature2D {
        @opencv_core.Ptr
        public static native AffineFeature2D create(@opencv_core.Ptr @Cast({"cv::FeatureDetector*"}) opencv_features2d.Feature2D feature2D);

        @opencv_core.Ptr
        public static native AffineFeature2D create(@opencv_core.Ptr @Cast({"cv::FeatureDetector*"}) opencv_features2d.Feature2D feature2D, @opencv_core.Ptr @Cast({"cv::DescriptorExtractor*"}) opencv_features2d.Feature2D feature2D2);

        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @StdVector Elliptic_KeyPoint elliptic_KeyPoint);

        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @StdVector Elliptic_KeyPoint elliptic_KeyPoint, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat2);

        public native void detect(@ByVal opencv_core.Mat mat, @StdVector Elliptic_KeyPoint elliptic_KeyPoint);

        public native void detect(@ByVal opencv_core.Mat mat, @StdVector Elliptic_KeyPoint elliptic_KeyPoint, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat2);

        public native void detect(@ByVal opencv_core.UMat uMat, @StdVector Elliptic_KeyPoint elliptic_KeyPoint);

        public native void detect(@ByVal opencv_core.UMat uMat, @StdVector Elliptic_KeyPoint elliptic_KeyPoint, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat2);

        public native void detectAndCompute(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @StdVector Elliptic_KeyPoint elliptic_KeyPoint, @ByVal opencv_core.GpuMat gpuMat3);

        public native void detectAndCompute(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @StdVector Elliptic_KeyPoint elliptic_KeyPoint, @ByVal opencv_core.GpuMat gpuMat3, @Cast({"bool"}) boolean z);

        public native void detectAndCompute(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @StdVector Elliptic_KeyPoint elliptic_KeyPoint, @ByVal opencv_core.Mat mat3);

        public native void detectAndCompute(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @StdVector Elliptic_KeyPoint elliptic_KeyPoint, @ByVal opencv_core.Mat mat3, @Cast({"bool"}) boolean z);

        public native void detectAndCompute(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @StdVector Elliptic_KeyPoint elliptic_KeyPoint, @ByVal opencv_core.UMat uMat3);

        public native void detectAndCompute(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @StdVector Elliptic_KeyPoint elliptic_KeyPoint, @ByVal opencv_core.UMat uMat3, @Cast({"bool"}) boolean z);

        static {
            Loader.load();
        }

        public AffineFeature2D(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::xfeatures2d")
    public static class SIFT extends opencv_features2d.Feature2D {
        private native void allocate();

        private native void allocateArray(long j);

        @opencv_core.Ptr
        public static native SIFT create();

        @opencv_core.Ptr
        public static native SIFT create(int i, int i2, double d, double d2, double d3);

        static {
            Loader.load();
        }

        public SIFT() {
            super((Pointer) null);
            allocate();
        }

        public SIFT(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public SIFT(Pointer p) {
            super(p);
        }

        public SIFT position(long position) {
            return (SIFT) super.position(position);
        }
    }

    @Namespace("cv::xfeatures2d")
    public static class SURF extends opencv_features2d.Feature2D {
        @opencv_core.Ptr
        public static native SURF create();

        @opencv_core.Ptr
        public static native SURF create(double d, int i, int i2, @Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2);

        @Cast({"bool"})
        public native boolean getExtended();

        public native double getHessianThreshold();

        public native int getNOctaveLayers();

        public native int getNOctaves();

        @Cast({"bool"})
        public native boolean getUpright();

        public native void setExtended(@Cast({"bool"}) boolean z);

        public native void setHessianThreshold(double d);

        public native void setNOctaveLayers(int i);

        public native void setNOctaves(int i);

        public native void setUpright(@Cast({"bool"}) boolean z);

        static {
            Loader.load();
        }

        public SURF(Pointer p) {
            super(p);
        }
    }
}
