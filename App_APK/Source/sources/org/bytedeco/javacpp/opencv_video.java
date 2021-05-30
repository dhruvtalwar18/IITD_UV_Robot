package org.bytedeco.javacpp;

import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Const;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.annotation.NoOffset;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.presets.opencv_core;

public class opencv_video extends org.bytedeco.javacpp.helper.opencv_video {
    public static final int MOTION_AFFINE = 2;
    public static final int MOTION_EUCLIDEAN = 1;
    public static final int MOTION_HOMOGRAPHY = 3;
    public static final int MOTION_TRANSLATION = 0;
    public static final int OPTFLOW_FARNEBACK_GAUSSIAN = 256;
    public static final int OPTFLOW_LK_GET_MIN_EIGENVALS = 8;
    public static final int OPTFLOW_USE_INITIAL_FLOW = 4;

    @Namespace("cv")
    @ByVal
    public static native opencv_core.RotatedRect CamShift(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.Rect rect, @ByVal opencv_core.TermCriteria termCriteria);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.RotatedRect CamShift(@ByVal opencv_core.Mat mat, @ByRef opencv_core.Rect rect, @ByVal opencv_core.TermCriteria termCriteria);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.RotatedRect CamShift(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.Rect rect, @ByVal opencv_core.TermCriteria termCriteria);

    @Namespace("cv")
    public static native int buildOpticalFlowPyramid(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Size size, int i);

    @Namespace("cv")
    public static native int buildOpticalFlowPyramid(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Size size, int i, @Cast({"bool"}) boolean z, int i2, int i3, @Cast({"bool"}) boolean z2);

    @Namespace("cv")
    public static native int buildOpticalFlowPyramid(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Size size, int i);

    @Namespace("cv")
    public static native int buildOpticalFlowPyramid(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Size size, int i, @Cast({"bool"}) boolean z, int i2, int i3, @Cast({"bool"}) boolean z2);

    @Namespace("cv")
    public static native int buildOpticalFlowPyramid(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Size size, int i);

    @Namespace("cv")
    public static native int buildOpticalFlowPyramid(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Size size, int i, @Cast({"bool"}) boolean z, int i2, int i3, @Cast({"bool"}) boolean z2);

    @Namespace("cv")
    public static native int buildOpticalFlowPyramid(@ByVal opencv_core.Mat mat, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Size size, int i);

    @Namespace("cv")
    public static native int buildOpticalFlowPyramid(@ByVal opencv_core.Mat mat, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Size size, int i, @Cast({"bool"}) boolean z, int i2, int i3, @Cast({"bool"}) boolean z2);

    @Namespace("cv")
    public static native int buildOpticalFlowPyramid(@ByVal opencv_core.Mat mat, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Size size, int i);

    @Namespace("cv")
    public static native int buildOpticalFlowPyramid(@ByVal opencv_core.Mat mat, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Size size, int i, @Cast({"bool"}) boolean z, int i2, int i3, @Cast({"bool"}) boolean z2);

    @Namespace("cv")
    public static native int buildOpticalFlowPyramid(@ByVal opencv_core.Mat mat, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Size size, int i);

    @Namespace("cv")
    public static native int buildOpticalFlowPyramid(@ByVal opencv_core.Mat mat, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Size size, int i, @Cast({"bool"}) boolean z, int i2, int i3, @Cast({"bool"}) boolean z2);

    @Namespace("cv")
    public static native int buildOpticalFlowPyramid(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Size size, int i);

    @Namespace("cv")
    public static native int buildOpticalFlowPyramid(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Size size, int i, @Cast({"bool"}) boolean z, int i2, int i3, @Cast({"bool"}) boolean z2);

    @Namespace("cv")
    public static native int buildOpticalFlowPyramid(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Size size, int i);

    @Namespace("cv")
    public static native int buildOpticalFlowPyramid(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Size size, int i, @Cast({"bool"}) boolean z, int i2, int i3, @Cast({"bool"}) boolean z2);

    @Namespace("cv")
    public static native int buildOpticalFlowPyramid(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Size size, int i);

    @Namespace("cv")
    public static native int buildOpticalFlowPyramid(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Size size, int i, @Cast({"bool"}) boolean z, int i2, int i3, @Cast({"bool"}) boolean z2);

    @Namespace("cv")
    public static native void calcOpticalFlowFarneback(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, double d, int i, int i2, int i3, int i4, double d2, int i5);

    @Namespace("cv")
    public static native void calcOpticalFlowFarneback(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, double d, int i, int i2, int i3, int i4, double d2, int i5);

    @Namespace("cv")
    public static native void calcOpticalFlowFarneback(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, double d, int i, int i2, int i3, int i4, double d2, int i5);

    @Namespace("cv")
    public static native void calcOpticalFlowPyrLK(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6);

    @Namespace("cv")
    public static native void calcOpticalFlowPyrLK(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6, @ByVal(nullValue = "cv::Size(21,21)") opencv_core.Size size, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01)") opencv_core.TermCriteria termCriteria, int i2, double d);

    @Namespace("cv")
    public static native void calcOpticalFlowPyrLK(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6);

    @Namespace("cv")
    public static native void calcOpticalFlowPyrLK(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, @ByVal(nullValue = "cv::Size(21,21)") opencv_core.Size size, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01)") opencv_core.TermCriteria termCriteria, int i2, double d);

    @Namespace("cv")
    public static native void calcOpticalFlowPyrLK(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6);

    @Namespace("cv")
    public static native void calcOpticalFlowPyrLK(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6, @ByVal(nullValue = "cv::Size(21,21)") opencv_core.Size size, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01)") opencv_core.TermCriteria termCriteria, int i2, double d);

    @Namespace("cv")
    @opencv_core.Ptr
    public static native BackgroundSubtractorKNN createBackgroundSubtractorKNN();

    @Namespace("cv")
    @opencv_core.Ptr
    public static native BackgroundSubtractorKNN createBackgroundSubtractorKNN(int i, double d, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    @opencv_core.Ptr
    public static native BackgroundSubtractorMOG2 createBackgroundSubtractorMOG2();

    @Namespace("cv")
    @opencv_core.Ptr
    public static native BackgroundSubtractorMOG2 createBackgroundSubtractorMOG2(int i, double d, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    @Deprecated
    @ByVal
    public static native opencv_core.Mat estimateRigidTransform(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    @Deprecated
    @ByVal
    public static native opencv_core.Mat estimateRigidTransform(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    @Deprecated
    @ByVal
    public static native opencv_core.Mat estimateRigidTransform(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native double findTransformECC(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv")
    public static native double findTransformECC(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 50, 0.001)") opencv_core.TermCriteria termCriteria, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat4);

    @Namespace("cv")
    public static native double findTransformECC(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv")
    public static native double findTransformECC(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 50, 0.001)") opencv_core.TermCriteria termCriteria, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat4);

    @Namespace("cv")
    public static native double findTransformECC(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv")
    public static native double findTransformECC(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 50, 0.001)") opencv_core.TermCriteria termCriteria, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat4);

    @Namespace("cv")
    public static native int meanShift(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.Rect rect, @ByVal opencv_core.TermCriteria termCriteria);

    @Namespace("cv")
    public static native int meanShift(@ByVal opencv_core.Mat mat, @ByRef opencv_core.Rect rect, @ByVal opencv_core.TermCriteria termCriteria);

    @Namespace("cv")
    public static native int meanShift(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.Rect rect, @ByVal opencv_core.TermCriteria termCriteria);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat readOpticalFlow(@opencv_core.Str String str);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat readOpticalFlow(@opencv_core.Str BytePointer bytePointer);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean writeOpticalFlow(@opencv_core.Str String str, @ByVal opencv_core.GpuMat gpuMat);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean writeOpticalFlow(@opencv_core.Str String str, @ByVal opencv_core.Mat mat);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean writeOpticalFlow(@opencv_core.Str String str, @ByVal opencv_core.UMat uMat);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean writeOpticalFlow(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.GpuMat gpuMat);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean writeOpticalFlow(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.Mat mat);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean writeOpticalFlow(@opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.UMat uMat);

    static {
        Loader.load();
    }

    @Namespace("cv")
    @NoOffset
    public static class KalmanFilter extends Pointer {
        private native void allocate();

        private native void allocate(int i, int i2);

        private native void allocate(int i, int i2, int i3, int i4);

        private native void allocateArray(long j);

        @ByRef
        public native opencv_core.Mat controlMatrix();

        public native KalmanFilter controlMatrix(opencv_core.Mat mat);

        @ByRef
        @Const
        public native opencv_core.Mat correct(@ByRef @Const opencv_core.Mat mat);

        @ByRef
        public native opencv_core.Mat errorCovPost();

        public native KalmanFilter errorCovPost(opencv_core.Mat mat);

        @ByRef
        public native opencv_core.Mat errorCovPre();

        public native KalmanFilter errorCovPre(opencv_core.Mat mat);

        @ByRef
        public native opencv_core.Mat gain();

        public native KalmanFilter gain(opencv_core.Mat mat);

        public native void init(int i, int i2);

        public native void init(int i, int i2, int i3, int i4);

        @ByRef
        public native opencv_core.Mat measurementMatrix();

        public native KalmanFilter measurementMatrix(opencv_core.Mat mat);

        @ByRef
        public native opencv_core.Mat measurementNoiseCov();

        public native KalmanFilter measurementNoiseCov(opencv_core.Mat mat);

        @ByRef
        @Const
        public native opencv_core.Mat predict();

        @ByRef
        @Const
        public native opencv_core.Mat predict(@ByRef(nullValue = "cv::Mat()") @Const opencv_core.Mat mat);

        @ByRef
        public native opencv_core.Mat processNoiseCov();

        public native KalmanFilter processNoiseCov(opencv_core.Mat mat);

        @ByRef
        public native opencv_core.Mat statePost();

        public native KalmanFilter statePost(opencv_core.Mat mat);

        @ByRef
        public native opencv_core.Mat statePre();

        public native KalmanFilter statePre(opencv_core.Mat mat);

        @ByRef
        public native opencv_core.Mat temp1();

        public native KalmanFilter temp1(opencv_core.Mat mat);

        @ByRef
        public native opencv_core.Mat temp2();

        public native KalmanFilter temp2(opencv_core.Mat mat);

        @ByRef
        public native opencv_core.Mat temp3();

        public native KalmanFilter temp3(opencv_core.Mat mat);

        @ByRef
        public native opencv_core.Mat temp4();

        public native KalmanFilter temp4(opencv_core.Mat mat);

        @ByRef
        public native opencv_core.Mat temp5();

        public native KalmanFilter temp5(opencv_core.Mat mat);

        @ByRef
        public native opencv_core.Mat transitionMatrix();

        public native KalmanFilter transitionMatrix(opencv_core.Mat mat);

        static {
            Loader.load();
        }

        public KalmanFilter(Pointer p) {
            super(p);
        }

        public KalmanFilter(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public KalmanFilter position(long position) {
            return (KalmanFilter) super.position(position);
        }

        public KalmanFilter() {
            super((Pointer) null);
            allocate();
        }

        public KalmanFilter(int dynamParams, int measureParams, int controlParams, int type) {
            super((Pointer) null);
            allocate(dynamParams, measureParams, controlParams, type);
        }

        public KalmanFilter(int dynamParams, int measureParams) {
            super((Pointer) null);
            allocate(dynamParams, measureParams);
        }
    }

    @Namespace("cv")
    public static class DenseOpticalFlow extends opencv_core.Algorithm {
        public native void calc(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

        public native void calc(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

        public native void calc(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

        public native void collectGarbage();

        static {
            Loader.load();
        }

        public DenseOpticalFlow(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class SparseOpticalFlow extends opencv_core.Algorithm {
        public native void calc(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5);

        public native void calc(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat6);

        public native void calc(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5);

        public native void calc(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat6);

        public native void calc(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5);

        public native void calc(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat6);

        static {
            Loader.load();
        }

        public SparseOpticalFlow(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class FarnebackOpticalFlow extends DenseOpticalFlow {
        @opencv_core.Ptr
        public static native FarnebackOpticalFlow create();

        @opencv_core.Ptr
        public static native FarnebackOpticalFlow create(int i, double d, @Cast({"bool"}) boolean z, int i2, int i3, int i4, double d2, int i5);

        @Cast({"bool"})
        public native boolean getFastPyramids();

        public native int getFlags();

        public native int getNumIters();

        public native int getNumLevels();

        public native int getPolyN();

        public native double getPolySigma();

        public native double getPyrScale();

        public native int getWinSize();

        public native void setFastPyramids(@Cast({"bool"}) boolean z);

        public native void setFlags(int i);

        public native void setNumIters(int i);

        public native void setNumLevels(int i);

        public native void setPolyN(int i);

        public native void setPolySigma(double d);

        public native void setPyrScale(double d);

        public native void setWinSize(int i);

        static {
            Loader.load();
        }

        public FarnebackOpticalFlow(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class VariationalRefinement extends DenseOpticalFlow {
        @opencv_core.Ptr
        public static native VariationalRefinement create();

        public native void calcUV(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4);

        public native void calcUV(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4);

        public native void calcUV(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4);

        public native float getAlpha();

        public native float getDelta();

        public native int getFixedPointIterations();

        public native float getGamma();

        public native float getOmega();

        public native int getSorIterations();

        public native void setAlpha(float f);

        public native void setDelta(float f);

        public native void setFixedPointIterations(int i);

        public native void setGamma(float f);

        public native void setOmega(float f);

        public native void setSorIterations(int i);

        static {
            Loader.load();
        }

        public VariationalRefinement(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class DISOpticalFlow extends DenseOpticalFlow {
        public static final int PRESET_FAST = 1;
        public static final int PRESET_MEDIUM = 2;
        public static final int PRESET_ULTRAFAST = 0;

        @opencv_core.Ptr
        public static native DISOpticalFlow create();

        @opencv_core.Ptr
        public static native DISOpticalFlow create(int i);

        public native int getFinestScale();

        public native int getGradientDescentIterations();

        public native int getPatchSize();

        public native int getPatchStride();

        @Cast({"bool"})
        public native boolean getUseMeanNormalization();

        @Cast({"bool"})
        public native boolean getUseSpatialPropagation();

        public native float getVariationalRefinementAlpha();

        public native float getVariationalRefinementDelta();

        public native float getVariationalRefinementGamma();

        public native int getVariationalRefinementIterations();

        public native void setFinestScale(int i);

        public native void setGradientDescentIterations(int i);

        public native void setPatchSize(int i);

        public native void setPatchStride(int i);

        public native void setUseMeanNormalization(@Cast({"bool"}) boolean z);

        public native void setUseSpatialPropagation(@Cast({"bool"}) boolean z);

        public native void setVariationalRefinementAlpha(float f);

        public native void setVariationalRefinementDelta(float f);

        public native void setVariationalRefinementGamma(float f);

        public native void setVariationalRefinementIterations(int i);

        static {
            Loader.load();
        }

        public DISOpticalFlow(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class SparsePyrLKOpticalFlow extends SparseOpticalFlow {
        @opencv_core.Ptr
        public static native SparsePyrLKOpticalFlow create();

        @opencv_core.Ptr
        public static native SparsePyrLKOpticalFlow create(@ByVal(nullValue = "cv::Size(21, 21)") opencv_core.Size size, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01)") opencv_core.TermCriteria termCriteria, int i2, double d);

        public native int getFlags();

        public native int getMaxLevel();

        public native double getMinEigThreshold();

        @ByVal
        public native opencv_core.TermCriteria getTermCriteria();

        @ByVal
        public native opencv_core.Size getWinSize();

        public native void setFlags(int i);

        public native void setMaxLevel(int i);

        public native void setMinEigThreshold(double d);

        public native void setTermCriteria(@ByRef opencv_core.TermCriteria termCriteria);

        public native void setWinSize(@ByVal opencv_core.Size size);

        static {
            Loader.load();
        }

        public SparsePyrLKOpticalFlow(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class BackgroundSubtractor extends opencv_core.Algorithm {
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

        public BackgroundSubtractor(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class BackgroundSubtractorMOG2 extends BackgroundSubtractor {
        public native void apply(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void apply(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, double d);

        public native void apply(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void apply(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, double d);

        public native void apply(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void apply(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, double d);

        public native double getBackgroundRatio();

        public native double getComplexityReductionThreshold();

        @Cast({"bool"})
        public native boolean getDetectShadows();

        public native int getHistory();

        public native int getNMixtures();

        public native double getShadowThreshold();

        public native int getShadowValue();

        public native double getVarInit();

        public native double getVarMax();

        public native double getVarMin();

        public native double getVarThreshold();

        public native double getVarThresholdGen();

        public native void setBackgroundRatio(double d);

        public native void setComplexityReductionThreshold(double d);

        public native void setDetectShadows(@Cast({"bool"}) boolean z);

        public native void setHistory(int i);

        public native void setNMixtures(int i);

        public native void setShadowThreshold(double d);

        public native void setShadowValue(int i);

        public native void setVarInit(double d);

        public native void setVarMax(double d);

        public native void setVarMin(double d);

        public native void setVarThreshold(double d);

        public native void setVarThresholdGen(double d);

        static {
            Loader.load();
        }

        public BackgroundSubtractorMOG2(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class BackgroundSubtractorKNN extends BackgroundSubtractor {
        @Cast({"bool"})
        public native boolean getDetectShadows();

        public native double getDist2Threshold();

        public native int getHistory();

        public native int getNSamples();

        public native double getShadowThreshold();

        public native int getShadowValue();

        public native int getkNNSamples();

        public native void setDetectShadows(@Cast({"bool"}) boolean z);

        public native void setDist2Threshold(double d);

        public native void setHistory(int i);

        public native void setNSamples(int i);

        public native void setShadowThreshold(double d);

        public native void setShadowValue(int i);

        public native void setkNNSamples(int i);

        static {
            Loader.load();
        }

        public BackgroundSubtractorKNN(Pointer p) {
            super(p);
        }
    }
}
