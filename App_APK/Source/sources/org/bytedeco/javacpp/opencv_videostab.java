package org.bytedeco.javacpp;

import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Const;
import org.bytedeco.javacpp.annotation.Name;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.annotation.NoOffset;
import org.bytedeco.javacpp.annotation.StdVector;
import org.bytedeco.javacpp.annotation.Virtual;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.opencv_features2d;
import org.bytedeco.javacpp.presets.opencv_core;

public class opencv_videostab extends org.bytedeco.javacpp.presets.opencv_videostab {
    public static final int MM_AFFINE = 5;
    public static final int MM_HOMOGRAPHY = 6;
    public static final int MM_RIGID = 3;
    public static final int MM_ROTATION = 2;
    public static final int MM_SIMILARITY = 4;
    public static final int MM_TRANSLATION = 0;
    public static final int MM_TRANSLATION_AND_SCALE = 1;
    public static final int MM_UNKNOWN = 7;

    @Namespace("cv::videostab")
    public static native float calcBlurriness(@ByRef @Const opencv_core.Mat mat);

    @Namespace("cv::videostab")
    public static native void calcFlowMask(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2, @ByRef @Const opencv_core.Mat mat3, float f, @ByRef @Const opencv_core.Mat mat4, @ByRef @Const opencv_core.Mat mat5, @ByRef opencv_core.Mat mat6);

    @Namespace("cv::videostab")
    public static native void completeFrameAccordingToFlow(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2, @ByRef @Const opencv_core.Mat mat3, @ByRef @Const opencv_core.Mat mat4, @ByRef @Const opencv_core.Mat mat5, float f, @ByRef opencv_core.Mat mat6, @ByRef opencv_core.Mat mat7);

    @Namespace("cv::videostab")
    @ByVal
    public static native opencv_core.Mat ensureInclusionConstraint(@ByRef @Const opencv_core.Mat mat, @ByVal opencv_core.Size size, float f);

    @Namespace("cv::videostab")
    @ByVal
    public static native opencv_core.Mat estimateGlobalMotionLeastSquares(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::videostab")
    @ByVal
    public static native opencv_core.Mat estimateGlobalMotionLeastSquares(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, FloatBuffer floatBuffer);

    @Namespace("cv::videostab")
    @ByVal
    public static native opencv_core.Mat estimateGlobalMotionLeastSquares(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, FloatPointer floatPointer);

    @Namespace("cv::videostab")
    @ByVal
    public static native opencv_core.Mat estimateGlobalMotionLeastSquares(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, float[] fArr);

    @Namespace("cv::videostab")
    @ByVal
    public static native opencv_core.Mat estimateGlobalMotionLeastSquares(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::videostab")
    @ByVal
    public static native opencv_core.Mat estimateGlobalMotionLeastSquares(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, FloatBuffer floatBuffer);

    @Namespace("cv::videostab")
    @ByVal
    public static native opencv_core.Mat estimateGlobalMotionLeastSquares(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, FloatPointer floatPointer);

    @Namespace("cv::videostab")
    @ByVal
    public static native opencv_core.Mat estimateGlobalMotionLeastSquares(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, float[] fArr);

    @Namespace("cv::videostab")
    @ByVal
    public static native opencv_core.Mat estimateGlobalMotionLeastSquares(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::videostab")
    @ByVal
    public static native opencv_core.Mat estimateGlobalMotionLeastSquares(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, FloatBuffer floatBuffer);

    @Namespace("cv::videostab")
    @ByVal
    public static native opencv_core.Mat estimateGlobalMotionLeastSquares(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, FloatPointer floatPointer);

    @Namespace("cv::videostab")
    @ByVal
    public static native opencv_core.Mat estimateGlobalMotionLeastSquares(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, float[] fArr);

    @Namespace("cv::videostab")
    @ByVal
    public static native opencv_core.Mat estimateGlobalMotionRansac(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::videostab")
    @ByVal
    public static native opencv_core.Mat estimateGlobalMotionRansac(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, @ByRef(nullValue = "cv::videostab::RansacParams::default2dMotion(cv::videostab::MM_AFFINE)") @Const RansacParams ransacParams, FloatBuffer floatBuffer, IntBuffer intBuffer);

    @Namespace("cv::videostab")
    @ByVal
    public static native opencv_core.Mat estimateGlobalMotionRansac(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, @ByRef(nullValue = "cv::videostab::RansacParams::default2dMotion(cv::videostab::MM_AFFINE)") @Const RansacParams ransacParams, FloatPointer floatPointer, IntPointer intPointer);

    @Namespace("cv::videostab")
    @ByVal
    public static native opencv_core.Mat estimateGlobalMotionRansac(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, @ByRef(nullValue = "cv::videostab::RansacParams::default2dMotion(cv::videostab::MM_AFFINE)") @Const RansacParams ransacParams, float[] fArr, int[] iArr);

    @Namespace("cv::videostab")
    @ByVal
    public static native opencv_core.Mat estimateGlobalMotionRansac(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::videostab")
    @ByVal
    public static native opencv_core.Mat estimateGlobalMotionRansac(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, @ByRef(nullValue = "cv::videostab::RansacParams::default2dMotion(cv::videostab::MM_AFFINE)") @Const RansacParams ransacParams, FloatBuffer floatBuffer, IntBuffer intBuffer);

    @Namespace("cv::videostab")
    @ByVal
    public static native opencv_core.Mat estimateGlobalMotionRansac(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, @ByRef(nullValue = "cv::videostab::RansacParams::default2dMotion(cv::videostab::MM_AFFINE)") @Const RansacParams ransacParams, FloatPointer floatPointer, IntPointer intPointer);

    @Namespace("cv::videostab")
    @ByVal
    public static native opencv_core.Mat estimateGlobalMotionRansac(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, @ByRef(nullValue = "cv::videostab::RansacParams::default2dMotion(cv::videostab::MM_AFFINE)") @Const RansacParams ransacParams, float[] fArr, int[] iArr);

    @Namespace("cv::videostab")
    @ByVal
    public static native opencv_core.Mat estimateGlobalMotionRansac(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::videostab")
    @ByVal
    public static native opencv_core.Mat estimateGlobalMotionRansac(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, @ByRef(nullValue = "cv::videostab::RansacParams::default2dMotion(cv::videostab::MM_AFFINE)") @Const RansacParams ransacParams, FloatBuffer floatBuffer, IntBuffer intBuffer);

    @Namespace("cv::videostab")
    @ByVal
    public static native opencv_core.Mat estimateGlobalMotionRansac(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, @ByRef(nullValue = "cv::videostab::RansacParams::default2dMotion(cv::videostab::MM_AFFINE)") @Const RansacParams ransacParams, FloatPointer floatPointer, IntPointer intPointer);

    @Namespace("cv::videostab")
    @ByVal
    public static native opencv_core.Mat estimateGlobalMotionRansac(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, @ByRef(nullValue = "cv::videostab::RansacParams::default2dMotion(cv::videostab::MM_AFFINE)") @Const RansacParams ransacParams, float[] fArr, int[] iArr);

    @Namespace("cv::videostab")
    public static native float estimateOptimalTrimRatio(@ByRef @Const opencv_core.Mat mat, @ByVal opencv_core.Size size);

    @Namespace("cv::videostab")
    @ByVal
    public static native opencv_core.Mat getMotion(int i, int i2, @ByRef @Const opencv_core.MatVector matVector);

    static {
        Loader.load();
    }

    @Namespace("cv::videostab")
    public static class IFrameSource extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        @Virtual(true)
        @ByVal
        public native opencv_core.Mat nextFrame();

        @Virtual(true)
        public native void reset();

        static {
            Loader.load();
        }

        public IFrameSource() {
            super((Pointer) null);
            allocate();
        }

        public IFrameSource(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public IFrameSource(Pointer p) {
            super(p);
        }

        public IFrameSource position(long position) {
            return (IFrameSource) super.position(position);
        }
    }

    @Namespace("cv::videostab")
    public static class NullFrameSource extends IFrameSource {
        private native void allocate();

        private native void allocateArray(long j);

        @ByVal
        public native opencv_core.Mat nextFrame();

        public native void reset();

        static {
            Loader.load();
        }

        public NullFrameSource() {
            super((Pointer) null);
            allocate();
        }

        public NullFrameSource(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public NullFrameSource(Pointer p) {
            super(p);
        }

        public NullFrameSource position(long position) {
            return (NullFrameSource) super.position(position);
        }
    }

    @Namespace("cv::videostab")
    @NoOffset
    public static class VideoFileSource extends IFrameSource {
        private native void allocate(@opencv_core.Str String str);

        private native void allocate(@opencv_core.Str String str, @Cast({"bool"}) boolean z);

        private native void allocate(@opencv_core.Str BytePointer bytePointer);

        private native void allocate(@opencv_core.Str BytePointer bytePointer, @Cast({"bool"}) boolean z);

        public native int count();

        public native double fps();

        public native int height();

        @ByVal
        public native opencv_core.Mat nextFrame();

        public native void reset();

        public native int width();

        static {
            Loader.load();
        }

        public VideoFileSource(Pointer p) {
            super(p);
        }

        public VideoFileSource(@opencv_core.Str BytePointer path, @Cast({"bool"}) boolean volatileFrame) {
            super((Pointer) null);
            allocate(path, volatileFrame);
        }

        public VideoFileSource(@opencv_core.Str BytePointer path) {
            super((Pointer) null);
            allocate(path);
        }

        public VideoFileSource(@opencv_core.Str String path, @Cast({"bool"}) boolean volatileFrame) {
            super((Pointer) null);
            allocate(path, volatileFrame);
        }

        public VideoFileSource(@opencv_core.Str String path) {
            super((Pointer) null);
            allocate(path);
        }
    }

    @Namespace("cv::videostab")
    public static class ILog extends Pointer {
        public native void print(String str);

        public native void print(@Cast({"const char*"}) BytePointer bytePointer);

        static {
            Loader.load();
        }

        public ILog(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::videostab")
    public static class NullLog extends ILog {
        private native void allocate();

        private native void allocateArray(long j);

        public native void print(String str);

        public native void print(@Cast({"const char*"}) BytePointer bytePointer);

        static {
            Loader.load();
        }

        public NullLog() {
            super((Pointer) null);
            allocate();
        }

        public NullLog(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public NullLog(Pointer p) {
            super(p);
        }

        public NullLog position(long position) {
            return (NullLog) super.position(position);
        }
    }

    @Namespace("cv::videostab")
    public static class LogToStdout extends ILog {
        private native void allocate();

        private native void allocateArray(long j);

        public native void print(String str);

        public native void print(@Cast({"const char*"}) BytePointer bytePointer);

        static {
            Loader.load();
        }

        public LogToStdout() {
            super((Pointer) null);
            allocate();
        }

        public LogToStdout(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public LogToStdout(Pointer p) {
            super(p);
        }

        public LogToStdout position(long position) {
            return (LogToStdout) super.position(position);
        }
    }

    @Namespace("cv::videostab")
    @NoOffset
    public static class FastMarchingMethod extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        @ByVal
        public native opencv_core.Mat distanceMap();

        static {
            Loader.load();
        }

        public FastMarchingMethod(Pointer p) {
            super(p);
        }

        public FastMarchingMethod(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public FastMarchingMethod position(long position) {
            return (FastMarchingMethod) super.position(position);
        }

        public FastMarchingMethod() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::videostab")
    public static class ISparseOptFlowEstimator extends Pointer {
        public native void run(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6);

        public native void run(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6);

        public native void run(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6);

        static {
            Loader.load();
        }

        public ISparseOptFlowEstimator(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::videostab")
    public static class IDenseOptFlowEstimator extends Pointer {
        public native void run(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5);

        public native void run(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5);

        public native void run(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5);

        static {
            Loader.load();
        }

        public IDenseOptFlowEstimator(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::videostab")
    @NoOffset
    public static class PyrLkOptFlowEstimatorBase extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        public native int maxLevel();

        public native void setMaxLevel(int i);

        public native void setWinSize(@ByVal opencv_core.Size size);

        @ByVal
        public native opencv_core.Size winSize();

        static {
            Loader.load();
        }

        public PyrLkOptFlowEstimatorBase(Pointer p) {
            super(p);
        }

        public PyrLkOptFlowEstimatorBase(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public PyrLkOptFlowEstimatorBase position(long position) {
            return (PyrLkOptFlowEstimatorBase) super.position(position);
        }

        public PyrLkOptFlowEstimatorBase() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::videostab")
    public static class SparsePyrLkOptFlowEstimator extends PyrLkOptFlowEstimatorBase {
        private native void allocate();

        private native void allocateArray(long j);

        @Namespace
        @Name({"static_cast<cv::videostab::ISparseOptFlowEstimator*>"})
        public static native ISparseOptFlowEstimator asISparseOptFlowEstimator(SparsePyrLkOptFlowEstimator sparsePyrLkOptFlowEstimator);

        public native void run(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6);

        public native void run(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6);

        public native void run(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6);

        static {
            Loader.load();
        }

        public SparsePyrLkOptFlowEstimator() {
            super((Pointer) null);
            allocate();
        }

        public SparsePyrLkOptFlowEstimator(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public SparsePyrLkOptFlowEstimator(Pointer p) {
            super(p);
        }

        public SparsePyrLkOptFlowEstimator position(long position) {
            return (SparsePyrLkOptFlowEstimator) super.position(position);
        }

        public ISparseOptFlowEstimator asISparseOptFlowEstimator() {
            return asISparseOptFlowEstimator(this);
        }
    }

    @Namespace("cv::videostab")
    @NoOffset
    public static class RansacParams extends Pointer {
        private native void allocate();

        private native void allocate(int i, float f, float f2, float f3);

        private native void allocateArray(long j);

        @ByVal
        public static native RansacParams default2dMotion(@Cast({"cv::videostab::MotionModel"}) int i);

        public native float eps();

        public native RansacParams eps(float f);

        public native int niters();

        public native float prob();

        public native RansacParams prob(float f);

        public native int size();

        public native RansacParams size(int i);

        public native float thresh();

        public native RansacParams thresh(float f);

        static {
            Loader.load();
        }

        public RansacParams(Pointer p) {
            super(p);
        }

        public RansacParams(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public RansacParams position(long position) {
            return (RansacParams) super.position(position);
        }

        public RansacParams() {
            super((Pointer) null);
            allocate();
        }

        public RansacParams(int size, float thresh, float eps, float prob) {
            super((Pointer) null);
            allocate(size, thresh, eps, prob);
        }
    }

    @Namespace("cv::videostab")
    public static class IOutlierRejector extends Pointer {
        public native void process(@ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

        public native void process(@ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

        public native void process(@ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

        static {
            Loader.load();
        }

        public IOutlierRejector(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::videostab")
    public static class NullOutlierRejector extends IOutlierRejector {
        private native void allocate();

        private native void allocateArray(long j);

        public native void process(@ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

        public native void process(@ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

        public native void process(@ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

        static {
            Loader.load();
        }

        public NullOutlierRejector() {
            super((Pointer) null);
            allocate();
        }

        public NullOutlierRejector(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public NullOutlierRejector(Pointer p) {
            super(p);
        }

        public NullOutlierRejector position(long position) {
            return (NullOutlierRejector) super.position(position);
        }
    }

    @Namespace("cv::videostab")
    @NoOffset
    public static class TranslationBasedLocalOutlierRejector extends IOutlierRejector {
        private native void allocate();

        private native void allocateArray(long j);

        @ByVal
        public native opencv_core.Size cellSize();

        public native void process(@ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

        public native void process(@ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

        public native void process(@ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

        @ByVal
        public native RansacParams ransacParams();

        public native void setCellSize(@ByVal opencv_core.Size size);

        public native void setRansacParams(@ByVal RansacParams ransacParams);

        static {
            Loader.load();
        }

        public TranslationBasedLocalOutlierRejector(Pointer p) {
            super(p);
        }

        public TranslationBasedLocalOutlierRejector(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public TranslationBasedLocalOutlierRejector position(long position) {
            return (TranslationBasedLocalOutlierRejector) super.position(position);
        }

        public TranslationBasedLocalOutlierRejector() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::videostab")
    @NoOffset
    public static class MotionEstimatorBase extends Pointer {
        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @Cast({"bool*"}) BoolPointer boolPointer);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @Cast({"bool*"}) boolean[] zArr);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @Cast({"bool*"}) BoolPointer boolPointer);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @Cast({"bool*"}) boolean[] zArr);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @Cast({"bool*"}) BoolPointer boolPointer);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @Cast({"bool*"}) boolean[] zArr);

        @Cast({"cv::videostab::MotionModel"})
        public native int motionModel();

        public native void setMotionModel(@Cast({"cv::videostab::MotionModel"}) int i);

        static {
            Loader.load();
        }

        public MotionEstimatorBase(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::videostab")
    @NoOffset
    public static class MotionEstimatorRansacL2 extends MotionEstimatorBase {
        private native void allocate();

        private native void allocate(@Cast({"cv::videostab::MotionModel"}) int i);

        private native void allocateArray(long j);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @Cast({"bool*"}) BoolPointer boolPointer);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @Cast({"bool*"}) boolean[] zArr);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @Cast({"bool*"}) BoolPointer boolPointer);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @Cast({"bool*"}) boolean[] zArr);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @Cast({"bool*"}) BoolPointer boolPointer);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @Cast({"bool*"}) boolean[] zArr);

        public native float minInlierRatio();

        @ByVal
        public native RansacParams ransacParams();

        public native void setMinInlierRatio(float f);

        public native void setRansacParams(@ByRef @Const RansacParams ransacParams);

        static {
            Loader.load();
        }

        public MotionEstimatorRansacL2(Pointer p) {
            super(p);
        }

        public MotionEstimatorRansacL2(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public MotionEstimatorRansacL2 position(long position) {
            return (MotionEstimatorRansacL2) super.position(position);
        }

        public MotionEstimatorRansacL2(@Cast({"cv::videostab::MotionModel"}) int model) {
            super((Pointer) null);
            allocate(model);
        }

        public MotionEstimatorRansacL2() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::videostab")
    @NoOffset
    public static class MotionEstimatorL1 extends MotionEstimatorBase {
        private native void allocate();

        private native void allocate(@Cast({"cv::videostab::MotionModel"}) int i);

        private native void allocateArray(long j);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @Cast({"bool*"}) BoolPointer boolPointer);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @Cast({"bool*"}) boolean[] zArr);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @Cast({"bool*"}) BoolPointer boolPointer);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @Cast({"bool*"}) boolean[] zArr);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @Cast({"bool*"}) BoolPointer boolPointer);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @Cast({"bool*"}) boolean[] zArr);

        static {
            Loader.load();
        }

        public MotionEstimatorL1(Pointer p) {
            super(p);
        }

        public MotionEstimatorL1(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public MotionEstimatorL1 position(long position) {
            return (MotionEstimatorL1) super.position(position);
        }

        public MotionEstimatorL1(@Cast({"cv::videostab::MotionModel"}) int model) {
            super((Pointer) null);
            allocate(model);
        }

        public MotionEstimatorL1() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::videostab")
    @NoOffset
    public static class ImageMotionEstimatorBase extends Pointer {
        @ByVal
        public native opencv_core.Mat estimate(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2);

        @ByVal
        public native opencv_core.Mat estimate(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2, @Cast({"bool*"}) BoolPointer boolPointer);

        @ByVal
        public native opencv_core.Mat estimate(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2, @Cast({"bool*"}) boolean[] zArr);

        @Cast({"cv::videostab::MotionModel"})
        public native int motionModel();

        public native void setMotionModel(@Cast({"cv::videostab::MotionModel"}) int i);

        static {
            Loader.load();
        }

        public ImageMotionEstimatorBase(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::videostab")
    @NoOffset
    public static class FromFileMotionReader extends ImageMotionEstimatorBase {
        private native void allocate(@opencv_core.Str String str);

        private native void allocate(@opencv_core.Str BytePointer bytePointer);

        @ByVal
        public native opencv_core.Mat estimate(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2);

        @ByVal
        public native opencv_core.Mat estimate(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2, @Cast({"bool*"}) BoolPointer boolPointer);

        @ByVal
        public native opencv_core.Mat estimate(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2, @Cast({"bool*"}) boolean[] zArr);

        static {
            Loader.load();
        }

        public FromFileMotionReader(Pointer p) {
            super(p);
        }

        public FromFileMotionReader(@opencv_core.Str BytePointer path) {
            super((Pointer) null);
            allocate(path);
        }

        public FromFileMotionReader(@opencv_core.Str String path) {
            super((Pointer) null);
            allocate(path);
        }
    }

    @Namespace("cv::videostab")
    @NoOffset
    public static class ToFileMotionWriter extends ImageMotionEstimatorBase {
        private native void allocate(@opencv_core.Str String str, @opencv_core.Ptr ImageMotionEstimatorBase imageMotionEstimatorBase);

        private native void allocate(@opencv_core.Str BytePointer bytePointer, @opencv_core.Ptr ImageMotionEstimatorBase imageMotionEstimatorBase);

        @ByVal
        public native opencv_core.Mat estimate(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2);

        @ByVal
        public native opencv_core.Mat estimate(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2, @Cast({"bool*"}) BoolPointer boolPointer);

        @ByVal
        public native opencv_core.Mat estimate(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2, @Cast({"bool*"}) boolean[] zArr);

        @Cast({"cv::videostab::MotionModel"})
        public native int motionModel();

        public native void setMotionModel(@Cast({"cv::videostab::MotionModel"}) int i);

        static {
            Loader.load();
        }

        public ToFileMotionWriter(Pointer p) {
            super(p);
        }

        public ToFileMotionWriter(@opencv_core.Str BytePointer path, @opencv_core.Ptr ImageMotionEstimatorBase estimator) {
            super((Pointer) null);
            allocate(path, estimator);
        }

        public ToFileMotionWriter(@opencv_core.Str String path, @opencv_core.Ptr ImageMotionEstimatorBase estimator) {
            super((Pointer) null);
            allocate(path, estimator);
        }
    }

    @Namespace("cv::videostab")
    @NoOffset
    public static class KeypointBasedMotionEstimator extends ImageMotionEstimatorBase {
        private native void allocate(@opencv_core.Ptr MotionEstimatorBase motionEstimatorBase);

        @opencv_core.Ptr
        @Cast({"cv::FeatureDetector*"})
        public native opencv_features2d.Feature2D detector();

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @Cast({"bool*"}) BoolPointer boolPointer);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @Cast({"bool*"}) boolean[] zArr);

        @ByVal
        public native opencv_core.Mat estimate(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2);

        @ByVal
        public native opencv_core.Mat estimate(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2, @Cast({"bool*"}) BoolPointer boolPointer);

        @ByVal
        public native opencv_core.Mat estimate(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2, @Cast({"bool*"}) boolean[] zArr);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @Cast({"bool*"}) BoolPointer boolPointer);

        @ByVal
        public native opencv_core.Mat estimate(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @Cast({"bool*"}) boolean[] zArr);

        @Cast({"cv::videostab::MotionModel"})
        public native int motionModel();

        @opencv_core.Ptr
        public native ISparseOptFlowEstimator opticalFlowEstimator();

        @opencv_core.Ptr
        public native IOutlierRejector outlierRejector();

        public native void setDetector(@opencv_core.Ptr @Cast({"cv::FeatureDetector*"}) opencv_features2d.Feature2D feature2D);

        public native void setMotionModel(@Cast({"cv::videostab::MotionModel"}) int i);

        public native void setOpticalFlowEstimator(@opencv_core.Ptr ISparseOptFlowEstimator iSparseOptFlowEstimator);

        public native void setOutlierRejector(@opencv_core.Ptr IOutlierRejector iOutlierRejector);

        static {
            Loader.load();
        }

        public KeypointBasedMotionEstimator(Pointer p) {
            super(p);
        }

        public KeypointBasedMotionEstimator(@opencv_core.Ptr MotionEstimatorBase estimator) {
            super((Pointer) null);
            allocate(estimator);
        }
    }

    @Namespace("cv::videostab")
    public static class IMotionStabilizer extends Pointer {
        public native void stabilize(int i, @ByRef @Const opencv_core.MatVector matVector, @ByVal opencv_core.IntIntPair intIntPair, opencv_core.Mat mat);

        static {
            Loader.load();
        }

        public IMotionStabilizer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::videostab")
    public static class MotionStabilizationPipeline extends IMotionStabilizer {
        private native void allocate();

        private native void allocateArray(long j);

        @Cast({"bool"})
        public native boolean empty();

        public native void pushBack(@opencv_core.Ptr IMotionStabilizer iMotionStabilizer);

        public native void stabilize(int i, @ByRef @Const opencv_core.MatVector matVector, @ByVal opencv_core.IntIntPair intIntPair, opencv_core.Mat mat);

        static {
            Loader.load();
        }

        public MotionStabilizationPipeline() {
            super((Pointer) null);
            allocate();
        }

        public MotionStabilizationPipeline(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public MotionStabilizationPipeline(Pointer p) {
            super(p);
        }

        public MotionStabilizationPipeline position(long position) {
            return (MotionStabilizationPipeline) super.position(position);
        }
    }

    @Namespace("cv::videostab")
    public static class MotionFilterBase extends IMotionStabilizer {
        @ByVal
        public native opencv_core.Mat stabilize(int i, @ByRef @Const opencv_core.MatVector matVector, @ByVal opencv_core.IntIntPair intIntPair);

        public native void stabilize(int i, @ByRef @Const opencv_core.MatVector matVector, @ByVal opencv_core.IntIntPair intIntPair, opencv_core.Mat mat);

        static {
            Loader.load();
        }

        public MotionFilterBase(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::videostab")
    @NoOffset
    public static class GaussianMotionFilter extends MotionFilterBase {
        private native void allocate();

        private native void allocate(int i, float f);

        private native void allocateArray(long j);

        public native int radius();

        public native void setParams(int i);

        public native void setParams(int i, float f);

        @ByVal
        public native opencv_core.Mat stabilize(int i, @ByRef @Const opencv_core.MatVector matVector, @ByVal opencv_core.IntIntPair intIntPair);

        public native float stdev();

        static {
            Loader.load();
        }

        public GaussianMotionFilter(Pointer p) {
            super(p);
        }

        public GaussianMotionFilter(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public GaussianMotionFilter position(long position) {
            return (GaussianMotionFilter) super.position(position);
        }

        public GaussianMotionFilter(int radius, float stdev) {
            super((Pointer) null);
            allocate(radius, stdev);
        }

        public GaussianMotionFilter() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::videostab")
    @NoOffset
    public static class LpMotionStabilizer extends IMotionStabilizer {
        private native void allocate();

        private native void allocate(@Cast({"cv::videostab::MotionModel"}) int i);

        private native void allocateArray(long j);

        @ByVal
        public native opencv_core.Size frameSize();

        @Cast({"cv::videostab::MotionModel"})
        public native int motionModel();

        public native void setFrameSize(@ByVal opencv_core.Size size);

        public native void setMotionModel(@Cast({"cv::videostab::MotionModel"}) int i);

        public native void setTrimRatio(float f);

        public native void setWeight1(float f);

        public native void setWeight2(float f);

        public native void setWeight3(float f);

        public native void setWeight4(float f);

        public native void stabilize(int i, @ByRef @Const opencv_core.MatVector matVector, @ByVal opencv_core.IntIntPair intIntPair, opencv_core.Mat mat);

        public native float trimRatio();

        public native float weight1();

        public native float weight2();

        public native float weight3();

        public native float weight4();

        static {
            Loader.load();
        }

        public LpMotionStabilizer(Pointer p) {
            super(p);
        }

        public LpMotionStabilizer(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public LpMotionStabilizer position(long position) {
            return (LpMotionStabilizer) super.position(position);
        }

        public LpMotionStabilizer(@Cast({"cv::videostab::MotionModel"}) int model) {
            super((Pointer) null);
            allocate(model);
        }

        public LpMotionStabilizer() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::videostab")
    @NoOffset
    public static class InpainterBase extends Pointer {
        @ByRef
        @Const
        public native opencv_core.MatVector frames();

        public native void inpaint(int i, @ByRef opencv_core.Mat mat, @ByRef opencv_core.Mat mat2);

        @Cast({"cv::videostab::MotionModel"})
        public native int motionModel();

        @ByRef
        @Const
        public native opencv_core.MatVector motions();

        public native int radius();

        public native void setFrames(@ByRef @Const opencv_core.MatVector matVector);

        public native void setMotionModel(@Cast({"cv::videostab::MotionModel"}) int i);

        public native void setMotions(@ByRef @Const opencv_core.MatVector matVector);

        public native void setRadius(int i);

        public native void setStabilizationMotions(@ByRef @Const opencv_core.MatVector matVector);

        public native void setStabilizedFrames(@ByRef @Const opencv_core.MatVector matVector);

        @ByRef
        @Const
        public native opencv_core.MatVector stabilizationMotions();

        @ByRef
        @Const
        public native opencv_core.MatVector stabilizedFrames();

        static {
            Loader.load();
        }

        public InpainterBase(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::videostab")
    public static class NullInpainter extends InpainterBase {
        private native void allocate();

        private native void allocateArray(long j);

        public native void inpaint(int i, @ByRef opencv_core.Mat mat, @ByRef opencv_core.Mat mat2);

        static {
            Loader.load();
        }

        public NullInpainter() {
            super((Pointer) null);
            allocate();
        }

        public NullInpainter(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public NullInpainter(Pointer p) {
            super(p);
        }

        public NullInpainter position(long position) {
            return (NullInpainter) super.position(position);
        }
    }

    @Namespace("cv::videostab")
    public static class InpaintingPipeline extends InpainterBase {
        private native void allocate();

        private native void allocateArray(long j);

        @Cast({"bool"})
        public native boolean empty();

        public native void inpaint(int i, @ByRef opencv_core.Mat mat, @ByRef opencv_core.Mat mat2);

        public native void pushBack(@opencv_core.Ptr InpainterBase inpainterBase);

        public native void setFrames(@ByRef @Const opencv_core.MatVector matVector);

        public native void setMotionModel(@Cast({"cv::videostab::MotionModel"}) int i);

        public native void setMotions(@ByRef @Const opencv_core.MatVector matVector);

        public native void setRadius(int i);

        public native void setStabilizationMotions(@ByRef @Const opencv_core.MatVector matVector);

        public native void setStabilizedFrames(@ByRef @Const opencv_core.MatVector matVector);

        static {
            Loader.load();
        }

        public InpaintingPipeline() {
            super((Pointer) null);
            allocate();
        }

        public InpaintingPipeline(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public InpaintingPipeline(Pointer p) {
            super(p);
        }

        public InpaintingPipeline position(long position) {
            return (InpaintingPipeline) super.position(position);
        }
    }

    @Namespace("cv::videostab")
    @NoOffset
    public static class ConsistentMosaicInpainter extends InpainterBase {
        private native void allocate();

        private native void allocateArray(long j);

        public native void inpaint(int i, @ByRef opencv_core.Mat mat, @ByRef opencv_core.Mat mat2);

        public native void setStdevThresh(float f);

        public native float stdevThresh();

        static {
            Loader.load();
        }

        public ConsistentMosaicInpainter(Pointer p) {
            super(p);
        }

        public ConsistentMosaicInpainter(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public ConsistentMosaicInpainter position(long position) {
            return (ConsistentMosaicInpainter) super.position(position);
        }

        public ConsistentMosaicInpainter() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::videostab")
    @NoOffset
    public static class MotionInpainter extends InpainterBase {
        private native void allocate();

        private native void allocateArray(long j);

        public native int borderMode();

        public native float distThresh();

        public native float flowErrorThreshold();

        public native void inpaint(int i, @ByRef opencv_core.Mat mat, @ByRef opencv_core.Mat mat2);

        @opencv_core.Ptr
        public native IDenseOptFlowEstimator optFlowEstimator();

        public native void setBorderMode(int i);

        public native void setDistThreshold(float f);

        public native void setFlowErrorThreshold(float f);

        public native void setOptFlowEstimator(@opencv_core.Ptr IDenseOptFlowEstimator iDenseOptFlowEstimator);

        static {
            Loader.load();
        }

        public MotionInpainter(Pointer p) {
            super(p);
        }

        public MotionInpainter(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public MotionInpainter position(long position) {
            return (MotionInpainter) super.position(position);
        }

        public MotionInpainter() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::videostab")
    @NoOffset
    public static class ColorAverageInpainter extends InpainterBase {
        private native void allocate();

        private native void allocateArray(long j);

        public native void inpaint(int i, @ByRef opencv_core.Mat mat, @ByRef opencv_core.Mat mat2);

        static {
            Loader.load();
        }

        public ColorAverageInpainter() {
            super((Pointer) null);
            allocate();
        }

        public ColorAverageInpainter(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public ColorAverageInpainter(Pointer p) {
            super(p);
        }

        public ColorAverageInpainter position(long position) {
            return (ColorAverageInpainter) super.position(position);
        }
    }

    @Namespace("cv::videostab")
    @NoOffset
    public static class ColorInpainter extends InpainterBase {
        private native void allocate();

        private native void allocate(int i, double d);

        private native void allocateArray(long j);

        public native void inpaint(int i, @ByRef opencv_core.Mat mat, @ByRef opencv_core.Mat mat2);

        static {
            Loader.load();
        }

        public ColorInpainter(Pointer p) {
            super(p);
        }

        public ColorInpainter(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public ColorInpainter position(long position) {
            return (ColorInpainter) super.position(position);
        }

        public ColorInpainter(int method, double radius) {
            super((Pointer) null);
            allocate(method, radius);
        }

        public ColorInpainter() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::videostab")
    @NoOffset
    public static class DeblurerBase extends Pointer {
        @StdVector
        public native FloatPointer blurrinessRates();

        public native void deblur(int i, @ByRef opencv_core.Mat mat);

        @ByRef
        @Const
        public native opencv_core.MatVector frames();

        @ByRef
        @Const
        public native opencv_core.MatVector motions();

        public native int radius();

        public native void setBlurrinessRates(@StdVector FloatBuffer floatBuffer);

        public native void setBlurrinessRates(@StdVector FloatPointer floatPointer);

        public native void setBlurrinessRates(@StdVector float[] fArr);

        public native void setFrames(@ByRef @Const opencv_core.MatVector matVector);

        public native void setMotions(@ByRef @Const opencv_core.MatVector matVector);

        public native void setRadius(int i);

        static {
            Loader.load();
        }

        public DeblurerBase(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::videostab")
    public static class NullDeblurer extends DeblurerBase {
        private native void allocate();

        private native void allocateArray(long j);

        public native void deblur(int i, @ByRef opencv_core.Mat mat);

        static {
            Loader.load();
        }

        public NullDeblurer() {
            super((Pointer) null);
            allocate();
        }

        public NullDeblurer(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public NullDeblurer(Pointer p) {
            super(p);
        }

        public NullDeblurer position(long position) {
            return (NullDeblurer) super.position(position);
        }
    }

    @Namespace("cv::videostab")
    @NoOffset
    public static class WeightingDeblurer extends DeblurerBase {
        private native void allocate();

        private native void allocateArray(long j);

        public native void deblur(int i, @ByRef opencv_core.Mat mat);

        public native float sensitivity();

        public native void setSensitivity(float f);

        static {
            Loader.load();
        }

        public WeightingDeblurer(Pointer p) {
            super(p);
        }

        public WeightingDeblurer(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public WeightingDeblurer position(long position) {
            return (WeightingDeblurer) super.position(position);
        }

        public WeightingDeblurer() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::videostab")
    @NoOffset
    public static class WobbleSuppressorBase extends Pointer {
        public native int frameCount();

        @opencv_core.Ptr
        public native ImageMotionEstimatorBase motionEstimator();

        @ByRef
        @Const
        public native opencv_core.MatVector motions();

        @ByRef
        @Const
        public native opencv_core.MatVector motions2();

        public native void setFrameCount(int i);

        public native void setMotionEstimator(@opencv_core.Ptr ImageMotionEstimatorBase imageMotionEstimatorBase);

        public native void setMotions(@ByRef @Const opencv_core.MatVector matVector);

        public native void setMotions2(@ByRef @Const opencv_core.MatVector matVector);

        public native void setStabilizationMotions(@ByRef @Const opencv_core.MatVector matVector);

        @ByRef
        @Const
        public native opencv_core.MatVector stabilizationMotions();

        public native void suppress(int i, @ByRef @Const opencv_core.Mat mat, @ByRef opencv_core.Mat mat2);

        static {
            Loader.load();
        }

        public WobbleSuppressorBase(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::videostab")
    public static class NullWobbleSuppressor extends WobbleSuppressorBase {
        private native void allocate();

        private native void allocateArray(long j);

        public native void suppress(int i, @ByRef @Const opencv_core.Mat mat, @ByRef opencv_core.Mat mat2);

        static {
            Loader.load();
        }

        public NullWobbleSuppressor() {
            super((Pointer) null);
            allocate();
        }

        public NullWobbleSuppressor(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public NullWobbleSuppressor(Pointer p) {
            super(p);
        }

        public NullWobbleSuppressor position(long position) {
            return (NullWobbleSuppressor) super.position(position);
        }
    }

    @Namespace("cv::videostab")
    @NoOffset
    public static class MoreAccurateMotionWobbleSuppressorBase extends WobbleSuppressorBase {
        public native int period();

        public native void setPeriod(int i);

        static {
            Loader.load();
        }

        public MoreAccurateMotionWobbleSuppressorBase(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::videostab")
    public static class MoreAccurateMotionWobbleSuppressor extends MoreAccurateMotionWobbleSuppressorBase {
        private native void allocate();

        private native void allocateArray(long j);

        public native void suppress(int i, @ByRef @Const opencv_core.Mat mat, @ByRef opencv_core.Mat mat2);

        static {
            Loader.load();
        }

        public MoreAccurateMotionWobbleSuppressor() {
            super((Pointer) null);
            allocate();
        }

        public MoreAccurateMotionWobbleSuppressor(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public MoreAccurateMotionWobbleSuppressor(Pointer p) {
            super(p);
        }

        public MoreAccurateMotionWobbleSuppressor position(long position) {
            return (MoreAccurateMotionWobbleSuppressor) super.position(position);
        }
    }

    @Namespace("cv::videostab")
    @NoOffset
    public static class StabilizerBase extends Pointer {
        public native int borderMode();

        @opencv_core.Ptr
        public native DeblurerBase deblurrer();

        @Cast({"bool"})
        public native boolean doCorrectionForInclusion();

        @opencv_core.Ptr
        public native IFrameSource frameSource();

        @opencv_core.Ptr
        public native InpainterBase inpainter();

        @opencv_core.Ptr
        public native ILog log();

        @opencv_core.Ptr
        public native ImageMotionEstimatorBase motionEstimator();

        public native int radius();

        public native void setBorderMode(int i);

        public native void setCorrectionForInclusion(@Cast({"bool"}) boolean z);

        public native void setDeblurer(@opencv_core.Ptr DeblurerBase deblurerBase);

        public native void setFrameSource(@opencv_core.Ptr IFrameSource iFrameSource);

        public native void setInpainter(@opencv_core.Ptr InpainterBase inpainterBase);

        public native void setLog(@opencv_core.Ptr ILog iLog);

        public native void setMotionEstimator(@opencv_core.Ptr ImageMotionEstimatorBase imageMotionEstimatorBase);

        public native void setRadius(int i);

        public native void setTrimRatio(float f);

        public native float trimRatio();

        static {
            Loader.load();
        }

        public StabilizerBase(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::videostab")
    @NoOffset
    public static class OnePassStabilizer extends StabilizerBase {
        private native void allocate();

        private native void allocateArray(long j);

        @Namespace
        @Name({"static_cast<cv::videostab::IFrameSource*>"})
        public static native IFrameSource asIFrameSource(OnePassStabilizer onePassStabilizer);

        @opencv_core.Ptr
        public native MotionFilterBase motionFilter();

        @ByVal
        public native opencv_core.Mat nextFrame();

        public native void reset();

        public native void setMotionFilter(@opencv_core.Ptr MotionFilterBase motionFilterBase);

        static {
            Loader.load();
        }

        public OnePassStabilizer(Pointer p) {
            super(p);
        }

        public OnePassStabilizer(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public OnePassStabilizer position(long position) {
            return (OnePassStabilizer) super.position(position);
        }

        public IFrameSource asIFrameSource() {
            return asIFrameSource(this);
        }

        public OnePassStabilizer() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::videostab")
    @NoOffset
    public static class TwoPassStabilizer extends StabilizerBase {
        private native void allocate();

        private native void allocateArray(long j);

        @Namespace
        @Name({"static_cast<cv::videostab::IFrameSource*>"})
        public static native IFrameSource asIFrameSource(TwoPassStabilizer twoPassStabilizer);

        @opencv_core.Ptr
        public native IMotionStabilizer motionStabilizer();

        @Cast({"bool"})
        public native boolean mustEstimateTrimaRatio();

        @ByVal
        public native opencv_core.Mat nextFrame();

        public native void reset();

        public native void setEstimateTrimRatio(@Cast({"bool"}) boolean z);

        public native void setMotionStabilizer(@opencv_core.Ptr IMotionStabilizer iMotionStabilizer);

        public native void setWobbleSuppressor(@opencv_core.Ptr WobbleSuppressorBase wobbleSuppressorBase);

        @opencv_core.Ptr
        public native WobbleSuppressorBase wobbleSuppressor();

        static {
            Loader.load();
        }

        public TwoPassStabilizer(Pointer p) {
            super(p);
        }

        public TwoPassStabilizer(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public TwoPassStabilizer position(long position) {
            return (TwoPassStabilizer) super.position(position);
        }

        public IFrameSource asIFrameSource() {
            return asIFrameSource(this);
        }

        public TwoPassStabilizer() {
            super((Pointer) null);
            allocate();
        }
    }
}
