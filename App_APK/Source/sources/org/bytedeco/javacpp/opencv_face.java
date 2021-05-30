package org.bytedeco.javacpp;

import java.nio.DoubleBuffer;
import java.nio.IntBuffer;
import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Const;
import org.bytedeco.javacpp.annotation.MemberGetter;
import org.bytedeco.javacpp.annotation.Name;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.annotation.NoOffset;
import org.bytedeco.javacpp.annotation.StdString;
import org.bytedeco.javacpp.annotation.StdVector;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.opencv_objdetect;
import org.bytedeco.javacpp.presets.opencv_core;

public class opencv_face extends org.bytedeco.javacpp.presets.opencv_face {
    @Namespace("cv::face")
    @opencv_core.Ptr
    public static native Facemark createFacemarkAAM();

    @Namespace("cv::face")
    @opencv_core.Ptr
    public static native Facemark createFacemarkKazemi();

    @Namespace("cv::face")
    @opencv_core.Ptr
    public static native Facemark createFacemarkLBF();

    @Namespace("cv::face")
    public static native void drawFacemarks(@ByVal opencv_core.Mat mat, @ByRef opencv_core.Point2fVector point2fVector, @ByVal(nullValue = "cv::Scalar(255,0,0)") opencv_core.Scalar scalar);

    @Namespace("cv::face")
    @Cast({"bool"})
    public static native boolean getFaces(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, CParams cParams);

    @Namespace("cv::face")
    @Cast({"bool"})
    public static native boolean getFacesHAAR(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @opencv_core.Str String str);

    @Namespace("cv::face")
    @Cast({"bool"})
    public static native boolean loadDatasetList(@opencv_core.Str String str, @opencv_core.Str String str2, @ByRef opencv_core.StringVector stringVector, @ByRef opencv_core.StringVector stringVector2);

    @Namespace("cv::face")
    @Cast({"bool"})
    public static native boolean loadDatasetList(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @ByRef opencv_core.StringVector stringVector, @ByRef opencv_core.StringVector stringVector2);

    @Namespace("cv::face")
    @Cast({"bool"})
    public static native boolean loadFacePoints(@opencv_core.Str String str, @ByRef opencv_core.Point2fVectorVector point2fVectorVector, float f);

    @Namespace("cv::face")
    @Cast({"bool"})
    public static native boolean loadTrainingData(@opencv_core.Str String str, @opencv_core.Str String str2, @ByRef opencv_core.StringVector stringVector, @ByRef opencv_core.Point2fVectorVector point2fVectorVector, float f);

    @Namespace("cv::face")
    @Cast({"bool"})
    public static native boolean loadTrainingData(@opencv_core.Str String str, @ByRef opencv_core.StringVector stringVector, @ByRef opencv_core.Point2fVectorVector point2fVectorVector, @Cast({"char"}) byte b, float f);

    @Namespace("cv::face")
    @Cast({"bool"})
    public static native boolean loadTrainingData(@ByVal opencv_core.StringVector stringVector, @ByRef opencv_core.Point2fVectorVector point2fVectorVector, @ByRef opencv_core.StringVector stringVector2);

    static {
        Loader.load();
    }

    @Namespace("cv::face")
    public static class PredictCollector extends Pointer {
        @Cast({"bool"})
        public native boolean collect(int i, double d);

        public native void init(@Cast({"size_t"}) long j);

        static {
            Loader.load();
        }

        public PredictCollector(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::face")
    @NoOffset
    public static class StandardCollector extends PredictCollector {
        private native void allocate();

        private native void allocate(double d);

        private native void allocateArray(long j);

        @opencv_core.Ptr
        public static native StandardCollector create();

        @opencv_core.Ptr
        public static native StandardCollector create(double d);

        @Cast({"bool"})
        public native boolean collect(int i, double d);

        public native double getMinDist();

        public native int getMinLabel();

        @ByVal
        public native opencv_core.IntDoublePairVector getResults();

        @ByVal
        public native opencv_core.IntDoublePairVector getResults(@Cast({"bool"}) boolean z);

        @ByVal
        public native opencv_core.IntDoubleMap getResultsMap();

        public native void init(@Cast({"size_t"}) long j);

        static {
            Loader.load();
        }

        public StandardCollector(Pointer p) {
            super(p);
        }

        public StandardCollector(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public StandardCollector position(long position) {
            return (StandardCollector) super.position(position);
        }

        @NoOffset
        public static class PredictResult extends Pointer {
            private native void allocate();

            private native void allocate(int i, double d);

            private native void allocateArray(long j);

            public native double distance();

            public native PredictResult distance(double d);

            public native int label();

            public native PredictResult label(int i);

            static {
                Loader.load();
            }

            public PredictResult(Pointer p) {
                super(p);
            }

            public PredictResult(long size) {
                super((Pointer) null);
                allocateArray(size);
            }

            public PredictResult position(long position) {
                return (PredictResult) super.position(position);
            }

            public PredictResult(int label_, double distance_) {
                super((Pointer) null);
                allocate(label_, distance_);
            }

            public PredictResult() {
                super((Pointer) null);
                allocate();
            }
        }

        public StandardCollector(double threshold_) {
            super((Pointer) null);
            allocate(threshold_);
        }

        public StandardCollector() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::face")
    @NoOffset
    public static class FaceRecognizer extends opencv_core.Algorithm {
        @Cast({"bool"})
        public native boolean empty();

        @opencv_core.Str
        public native BytePointer getLabelInfo(int i);

        @StdVector
        public native IntBuffer getLabelsByString(@opencv_core.Str String str);

        @StdVector
        public native IntPointer getLabelsByString(@opencv_core.Str BytePointer bytePointer);

        public native double getThreshold();

        public native void predict(@ByVal opencv_core.GpuMat gpuMat, @ByRef IntBuffer intBuffer, @ByRef DoubleBuffer doubleBuffer);

        public native void predict(@ByVal opencv_core.GpuMat gpuMat, @ByRef IntPointer intPointer, @ByRef DoublePointer doublePointer);

        public native void predict(@ByVal opencv_core.GpuMat gpuMat, @ByRef int[] iArr, @ByRef double[] dArr);

        public native void predict(@ByVal opencv_core.Mat mat, @ByRef IntBuffer intBuffer, @ByRef DoubleBuffer doubleBuffer);

        public native void predict(@ByVal opencv_core.Mat mat, @ByRef IntPointer intPointer, @ByRef DoublePointer doublePointer);

        public native void predict(@ByVal opencv_core.Mat mat, @ByRef int[] iArr, @ByRef double[] dArr);

        public native void predict(@ByVal opencv_core.UMat uMat, @ByRef IntBuffer intBuffer, @ByRef DoubleBuffer doubleBuffer);

        public native void predict(@ByVal opencv_core.UMat uMat, @ByRef IntPointer intPointer, @ByRef DoublePointer doublePointer);

        public native void predict(@ByVal opencv_core.UMat uMat, @ByRef int[] iArr, @ByRef double[] dArr);

        @Name({"predict"})
        public native void predict_collect(@ByVal opencv_core.GpuMat gpuMat, @opencv_core.Ptr PredictCollector predictCollector);

        @Name({"predict"})
        public native void predict_collect(@ByVal opencv_core.Mat mat, @opencv_core.Ptr PredictCollector predictCollector);

        @Name({"predict"})
        public native void predict_collect(@ByVal opencv_core.UMat uMat, @opencv_core.Ptr PredictCollector predictCollector);

        @Name({"predict"})
        public native int predict_label(@ByVal opencv_core.GpuMat gpuMat);

        @Name({"predict"})
        public native int predict_label(@ByVal opencv_core.Mat mat);

        @Name({"predict"})
        public native int predict_label(@ByVal opencv_core.UMat uMat);

        public native void read(@opencv_core.Str String str);

        public native void read(@opencv_core.Str BytePointer bytePointer);

        public native void read(@ByRef @Const opencv_core.FileNode fileNode);

        public native void setLabelInfo(int i, @opencv_core.Str String str);

        public native void setLabelInfo(int i, @opencv_core.Str BytePointer bytePointer);

        public native void setThreshold(double d);

        public native void train(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat);

        public native void train(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat);

        public native void train(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat);

        public native void train(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat);

        public native void train(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat);

        public native void train(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat);

        public native void train(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat);

        public native void train(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat);

        public native void train(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat);

        public native void update(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat);

        public native void update(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat);

        public native void update(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat);

        public native void update(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat);

        public native void update(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat);

        public native void update(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat);

        public native void update(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat);

        public native void update(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat);

        public native void update(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat);

        public native void write(@opencv_core.Str String str);

        public native void write(@opencv_core.Str BytePointer bytePointer);

        public native void write(@ByRef opencv_core.FileStorage fileStorage);

        static {
            Loader.load();
        }

        public FaceRecognizer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::face")
    @NoOffset
    public static class BasicFaceRecognizer extends FaceRecognizer {
        @Cast({"bool"})
        public native boolean empty();

        @ByVal
        public native opencv_core.Mat getEigenValues();

        @ByVal
        public native opencv_core.Mat getEigenVectors();

        @ByVal
        public native opencv_core.Mat getLabels();

        @ByVal
        public native opencv_core.Mat getMean();

        public native int getNumComponents();

        @ByVal
        public native opencv_core.MatVector getProjections();

        public native double getThreshold();

        public native void read(@ByRef @Const opencv_core.FileNode fileNode);

        public native void setNumComponents(int i);

        public native void setThreshold(double d);

        public native void write(@ByRef opencv_core.FileStorage fileStorage);

        static {
            Loader.load();
        }

        public BasicFaceRecognizer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::face")
    public static class EigenFaceRecognizer extends BasicFaceRecognizer {
        @opencv_core.Ptr
        public static native EigenFaceRecognizer create();

        @opencv_core.Ptr
        public static native EigenFaceRecognizer create(int i, double d);

        static {
            Loader.load();
        }

        public EigenFaceRecognizer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::face")
    public static class FisherFaceRecognizer extends BasicFaceRecognizer {
        @opencv_core.Ptr
        public static native FisherFaceRecognizer create();

        @opencv_core.Ptr
        public static native FisherFaceRecognizer create(int i, double d);

        static {
            Loader.load();
        }

        public FisherFaceRecognizer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::face")
    public static class LBPHFaceRecognizer extends FaceRecognizer {
        @opencv_core.Ptr
        public static native LBPHFaceRecognizer create();

        @opencv_core.Ptr
        public static native LBPHFaceRecognizer create(int i, int i2, int i3, int i4, double d);

        public native int getGridX();

        public native int getGridY();

        @ByVal
        public native opencv_core.MatVector getHistograms();

        @ByVal
        public native opencv_core.Mat getLabels();

        public native int getNeighbors();

        public native int getRadius();

        public native double getThreshold();

        public native void setGridX(int i);

        public native void setGridY(int i);

        public native void setNeighbors(int i);

        public native void setRadius(int i);

        public native void setThreshold(double d);

        static {
            Loader.load();
        }

        public LBPHFaceRecognizer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::face")
    public static class Facemark extends opencv_core.Algorithm {
        @Cast({"bool"})
        public native boolean fit(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @ByRef opencv_core.Point2fVectorVector point2fVectorVector);

        public native void loadModel(@opencv_core.Str String str);

        public native void loadModel(@opencv_core.Str BytePointer bytePointer);

        static {
            Loader.load();
        }

        public Facemark(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::face")
    @NoOffset
    public static class CParams extends Pointer {
        private native void allocate(@opencv_core.Str String str);

        private native void allocate(@opencv_core.Str String str, double d, int i, @ByVal(nullValue = "cv::Size(30, 30)") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2);

        private native void allocate(@opencv_core.Str BytePointer bytePointer);

        private native void allocate(@opencv_core.Str BytePointer bytePointer, double d, int i, @ByVal(nullValue = "cv::Size(30, 30)") opencv_core.Size size, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2);

        @opencv_core.Str
        public native BytePointer cascade();

        public native CParams cascade(BytePointer bytePointer);

        public native CParams face_cascade(opencv_objdetect.CascadeClassifier cascadeClassifier);

        @ByRef
        public native opencv_objdetect.CascadeClassifier face_cascade();

        @ByRef
        public native opencv_core.Size maxSize();

        public native CParams maxSize(opencv_core.Size size);

        public native int minNeighbors();

        public native CParams minNeighbors(int i);

        @ByRef
        public native opencv_core.Size minSize();

        public native CParams minSize(opencv_core.Size size);

        public native double scaleFactor();

        public native CParams scaleFactor(double d);

        static {
            Loader.load();
        }

        public CParams(Pointer p) {
            super(p);
        }

        public CParams(@opencv_core.Str BytePointer cascade_model, double sf, int minN, @ByVal(nullValue = "cv::Size(30, 30)") opencv_core.Size minSz, @ByVal(nullValue = "cv::Size()") opencv_core.Size maxSz) {
            super((Pointer) null);
            allocate(cascade_model, sf, minN, minSz, maxSz);
        }

        public CParams(@opencv_core.Str BytePointer cascade_model) {
            super((Pointer) null);
            allocate(cascade_model);
        }

        public CParams(@opencv_core.Str String cascade_model, double sf, int minN, @ByVal(nullValue = "cv::Size(30, 30)") opencv_core.Size minSz, @ByVal(nullValue = "cv::Size()") opencv_core.Size maxSz) {
            super((Pointer) null);
            allocate(cascade_model, sf, minN, minSz, maxSz);
        }

        public CParams(@opencv_core.Str String cascade_model) {
            super((Pointer) null);
            allocate(cascade_model);
        }
    }

    @Namespace("cv::face")
    public static class FacemarkTrain extends Facemark {
        @Cast({"bool"})
        public native boolean addTrainingSample(@ByVal opencv_core.Mat mat, @ByRef opencv_core.Point2fVector point2fVector);

        @Cast({"bool"})
        public native boolean getData();

        @Cast({"bool"})
        public native boolean getData(Pointer pointer);

        @Cast({"bool"})
        public native boolean getFaces(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector);

        @Cast({"bool"})
        public native boolean setFaceDetector(@Cast({"cv::face::FN_FaceDetector"}) Pointer pointer);

        @Cast({"bool"})
        public native boolean setFaceDetector(@Cast({"cv::face::FN_FaceDetector"}) Pointer pointer, Pointer pointer2);

        public native void training();

        public native void training(Pointer pointer);

        static {
            Loader.load();
        }

        public FacemarkTrain(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::face")
    public static class FacemarkLBF extends FacemarkTrain {
        @opencv_core.Ptr
        public static native FacemarkLBF create();

        @opencv_core.Ptr
        public static native FacemarkLBF create(@ByRef(nullValue = "cv::face::FacemarkLBF::Params()") @Const Params params);

        static {
            Loader.load();
        }

        public FacemarkLBF(Pointer p) {
            super(p);
        }

        @NoOffset
        public static class Params extends Pointer {
            private native void allocate();

            private native void allocateArray(long j);

            public native double bagging_overlap();

            public native Params bagging_overlap(double d);

            @opencv_core.Str
            public native BytePointer cascade_face();

            public native Params cascade_face(BytePointer bytePointer);

            @ByRef
            public native opencv_core.Rect detectROI();

            public native Params detectROI(opencv_core.Rect rect);

            @StdVector
            public native IntPointer feats_m();

            public native Params feats_m(IntPointer intPointer);

            public native int initShape_n();

            public native Params initShape_n(int i);

            @StdString
            public native BytePointer model_filename();

            public native Params model_filename(BytePointer bytePointer);

            public native int n_landmarks();

            public native Params n_landmarks(int i);

            @MemberGetter
            @StdVector
            public native IntPointer pupils();

            @StdVector
            public native IntPointer pupils(int i);

            public native Params pupils(int i, IntPointer intPointer);

            @StdVector
            public native DoublePointer radius_m();

            public native Params radius_m(DoublePointer doublePointer);

            public native void read(@ByRef @Const opencv_core.FileNode fileNode);

            public native Params save_model(boolean z);

            @Cast({"bool"})
            public native boolean save_model();

            @Cast({"unsigned int"})
            public native int seed();

            public native Params seed(int i);

            public native double shape_offset();

            public native Params shape_offset(double d);

            public native int stages_n();

            public native Params stages_n(int i);

            public native int tree_depth();

            public native Params tree_depth(int i);

            public native int tree_n();

            public native Params tree_n(int i);

            public native Params verbose(boolean z);

            @Cast({"bool"})
            public native boolean verbose();

            public native void write(@ByRef opencv_core.FileStorage fileStorage);

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

        @NoOffset
        public static class BBox extends Pointer {
            private native void allocate();

            private native void allocate(double d, double d2, double d3, double d4);

            private native void allocateArray(long j);

            public native double height();

            public native BBox height(double d);

            @ByVal
            public native opencv_core.Mat project(@ByRef @Const opencv_core.Mat mat);

            @ByVal
            public native opencv_core.Mat reproject(@ByRef @Const opencv_core.Mat mat);

            public native double width();

            public native BBox width(double d);

            public native double x();

            public native BBox x(double d);

            public native double x_center();

            public native BBox x_center(double d);

            public native double x_scale();

            public native BBox x_scale(double d);

            public native double y();

            public native BBox y(double d);

            public native double y_center();

            public native BBox y_center(double d);

            public native double y_scale();

            public native BBox y_scale(double d);

            static {
                Loader.load();
            }

            public BBox(Pointer p) {
                super(p);
            }

            public BBox(long size) {
                super((Pointer) null);
                allocateArray(size);
            }

            public BBox position(long position) {
                return (BBox) super.position(position);
            }

            public BBox() {
                super((Pointer) null);
                allocate();
            }

            public BBox(double x, double y, double w, double h) {
                super((Pointer) null);
                allocate(x, y, w, h);
            }
        }
    }

    @Namespace("cv::face")
    public static class FacemarkAAM extends FacemarkTrain {
        @opencv_core.Ptr
        public static native FacemarkAAM create();

        @opencv_core.Ptr
        public static native FacemarkAAM create(@ByRef(nullValue = "cv::face::FacemarkAAM::Params()") @Const Params params);

        @Cast({"bool"})
        public native boolean fitConfig(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @ByRef opencv_core.Point2fVectorVector point2fVectorVector, @StdVector Config config);

        static {
            Loader.load();
        }

        public FacemarkAAM(Pointer p) {
            super(p);
        }

        @NoOffset
        public static class Params extends Pointer {
            private native void allocate();

            private native void allocateArray(long j);

            public native int m();

            public native Params m(int i);

            public native int max_m();

            public native Params max_m(int i);

            public native int max_n();

            public native Params max_n(int i);

            @StdString
            public native BytePointer model_filename();

            public native Params model_filename(BytePointer bytePointer);

            public native int n();

            public native Params n(int i);

            public native int n_iter();

            public native Params n_iter(int i);

            public native void read(@ByRef @Const opencv_core.FileNode fileNode);

            public native Params save_model(boolean z);

            @Cast({"bool"})
            public native boolean save_model();

            @StdVector
            public native FloatPointer scales();

            public native Params scales(FloatPointer floatPointer);

            public native int texture_max_m();

            public native Params texture_max_m(int i);

            public native Params verbose(boolean z);

            @Cast({"bool"})
            public native boolean verbose();

            public native void write(@ByRef opencv_core.FileStorage fileStorage);

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

        @NoOffset
        public static class Config extends Pointer {
            private native void allocate();

            private native void allocate(@ByVal(nullValue = "cv::Mat::eye(2,2,CV_32F)") opencv_core.Mat mat, @ByVal(nullValue = "cv::Point2f(0.0f, 0.0f)") opencv_core.Point2f point2f, float f, int i);

            private native void allocateArray(long j);

            @ByRef
            public native opencv_core.Mat R();

            public native Config R(opencv_core.Mat mat);

            public native int model_scale_idx();

            public native Config model_scale_idx(int i);

            public native float scale();

            public native Config scale(float f);

            @ByRef
            public native opencv_core.Point2f t();

            public native Config t(opencv_core.Point2f point2f);

            static {
                Loader.load();
            }

            public Config(Pointer p) {
                super(p);
            }

            public Config(long size) {
                super((Pointer) null);
                allocateArray(size);
            }

            public Config position(long position) {
                return (Config) super.position(position);
            }

            public Config(@ByVal(nullValue = "cv::Mat::eye(2,2,CV_32F)") opencv_core.Mat rot, @ByVal(nullValue = "cv::Point2f(0.0f, 0.0f)") opencv_core.Point2f trans, float scaling, int scale_id) {
                super((Pointer) null);
                allocate(rot, trans, scaling, scale_id);
            }

            public Config() {
                super((Pointer) null);
                allocate();
            }
        }

        public static class Data extends Pointer {
            private native void allocate();

            private native void allocateArray(long j);

            @ByRef
            public native opencv_core.Point2fVector s0();

            public native Data s0(opencv_core.Point2fVector point2fVector);

            static {
                Loader.load();
            }

            public Data() {
                super((Pointer) null);
                allocate();
            }

            public Data(long size) {
                super((Pointer) null);
                allocateArray(size);
            }

            public Data(Pointer p) {
                super(p);
            }

            public Data position(long position) {
                return (Data) super.position(position);
            }
        }

        public static class Model extends Pointer {
            private native void allocate();

            private native void allocateArray(long j);

            @ByRef
            public native opencv_core.Mat Q();

            public native Model Q(opencv_core.Mat mat);

            @ByRef
            public native opencv_core.Mat S();

            public native Model S(opencv_core.Mat mat);

            @ByRef
            public native opencv_core.Point2fVector s0();

            public native Model s0(opencv_core.Point2fVector point2fVector);

            @StdVector
            public native FloatPointer scales();

            public native Model scales(FloatPointer floatPointer);

            @StdVector
            public native Texture textures();

            public native Model textures(Texture texture);

            @ByRef
            @Cast({"std::vector<cv::Vec3i>*"})
            public native opencv_core.Point3iVector triangles();

            public native Model triangles(opencv_core.Point3iVector point3iVector);

            static {
                Loader.load();
            }

            public Model() {
                super((Pointer) null);
                allocate();
            }

            public Model(long size) {
                super((Pointer) null);
                allocateArray(size);
            }

            public Model(Pointer p) {
                super(p);
            }

            public Model position(long position) {
                return (Model) super.position(position);
            }

            public static class Texture extends Pointer {
                private native void allocate();

                private native void allocateArray(long j);

                @ByRef
                public native opencv_core.Mat A();

                public native Texture A(opencv_core.Mat mat);

                @ByRef
                public native opencv_core.Mat A0();

                public native Texture A0(opencv_core.Mat mat);

                @ByRef
                public native opencv_core.Mat AA();

                public native Texture AA(opencv_core.Mat mat);

                @ByRef
                public native opencv_core.Mat AA0();

                public native Texture AA0(opencv_core.Mat mat);

                @ByRef
                public native opencv_core.Point2fVector base_shape();

                public native Texture base_shape(opencv_core.Point2fVector point2fVector);

                @StdVector
                public native IntPointer ind1();

                public native Texture ind1(IntPointer intPointer);

                @StdVector
                public native IntPointer ind2();

                public native Texture ind2(IntPointer intPointer);

                public native int max_m();

                public native Texture max_m(int i);

                @ByRef
                public native opencv_core.Rect resolution();

                public native Texture resolution(opencv_core.Rect rect);

                @ByRef
                public native opencv_core.PointVectorVector textureIdx();

                public native Texture textureIdx(opencv_core.PointVectorVector pointVectorVector);

                static {
                    Loader.load();
                }

                public Texture() {
                    super((Pointer) null);
                    allocate();
                }

                public Texture(long size) {
                    super((Pointer) null);
                    allocateArray(size);
                }

                public Texture(Pointer p) {
                    super(p);
                }

                public Texture position(long position) {
                    return (Texture) super.position(position);
                }
            }
        }
    }

    @Namespace("cv::face")
    public static class FacemarkKazemi extends Facemark {
        @opencv_core.Ptr
        public static native FacemarkKazemi create();

        @opencv_core.Ptr
        public static native FacemarkKazemi create(@ByRef(nullValue = "cv::face::FacemarkKazemi::Params()") @Const Params params);

        @Cast({"bool"})
        public native boolean getFaces(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector);

        @Cast({"bool"})
        public native boolean setFaceDetector(@Cast({"bool (*)(cv::InputArray, cv::OutputArray, void*)"}) Pointer pointer, Pointer pointer2);

        @Cast({"bool"})
        public native boolean training(@ByRef opencv_core.MatVector matVector, @ByRef opencv_core.Point2fVectorVector point2fVectorVector, @StdString String str, @ByVal opencv_core.Size size);

        @Cast({"bool"})
        public native boolean training(@ByRef opencv_core.MatVector matVector, @ByRef opencv_core.Point2fVectorVector point2fVectorVector, @StdString String str, @ByVal opencv_core.Size size, @StdString String str2);

        @Cast({"bool"})
        public native boolean training(@ByRef opencv_core.MatVector matVector, @ByRef opencv_core.Point2fVectorVector point2fVectorVector, @StdString BytePointer bytePointer, @ByVal opencv_core.Size size);

        @Cast({"bool"})
        public native boolean training(@ByRef opencv_core.MatVector matVector, @ByRef opencv_core.Point2fVectorVector point2fVectorVector, @StdString BytePointer bytePointer, @ByVal opencv_core.Size size, @StdString BytePointer bytePointer2);

        static {
            Loader.load();
        }

        public FacemarkKazemi(Pointer p) {
            super(p);
        }

        @NoOffset
        public static class Params extends Pointer {
            private native void allocate();

            private native void allocateArray(long j);

            @Cast({"unsigned long"})
            public native long cascade_depth();

            public native Params cascade_depth(long j);

            @opencv_core.Str
            public native BytePointer configfile();

            public native Params configfile(BytePointer bytePointer);

            public native float lambda();

            public native Params lambda(float f);

            public native float learning_rate();

            public native Params learning_rate(float f);

            @Cast({"unsigned long"})
            public native long num_test_coordinates();

            public native Params num_test_coordinates(long j);

            @Cast({"unsigned long"})
            public native long num_test_splits();

            public native Params num_test_splits(long j);

            @Cast({"unsigned long"})
            public native long num_trees_per_cascade_level();

            public native Params num_trees_per_cascade_level(long j);

            @Cast({"unsigned long"})
            public native long oversampling_amount();

            public native Params oversampling_amount(long j);

            @Cast({"unsigned long"})
            public native long tree_depth();

            public native Params tree_depth(long j);

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
