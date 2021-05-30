package org.bytedeco.javacpp;

import java.nio.DoubleBuffer;
import java.nio.IntBuffer;
import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Const;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.annotation.NoOffset;
import org.bytedeco.javacpp.annotation.StdVector;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.presets.opencv_core;

public class opencv_flann extends org.bytedeco.javacpp.presets.opencv_flann {
    public static final int AUTOTUNED = 255;
    public static final int CENTERS_GONZALES = 1;
    public static final int CENTERS_KMEANSPP = 2;
    public static final int CENTERS_RANDOM = 0;
    public static final int COMPOSITE = 3;
    public static final int CS = 7;
    public static final int EUCLIDEAN = 1;
    public static final int FLANN_CENTERS_GONZALES = 1;
    public static final int FLANN_CENTERS_GROUPWISE = 3;
    public static final int FLANN_CENTERS_KMEANSPP = 2;
    public static final int FLANN_CENTERS_RANDOM = 0;
    public static final int FLANN_CHECKS_AUTOTUNED = -2;
    public static final int FLANN_CHECKS_UNLIMITED = -1;
    public static final int FLANN_DIST_CHI_SQUARE = 7;
    public static final int FLANN_DIST_CS = 7;
    public static final int FLANN_DIST_EUCLIDEAN = 1;
    public static final int FLANN_DIST_HAMMING = 9;
    public static final int FLANN_DIST_HELLINGER = 6;
    public static final int FLANN_DIST_HIST_INTERSECT = 5;
    public static final int FLANN_DIST_KL = 8;
    public static final int FLANN_DIST_KULLBACK_LEIBLER = 8;
    public static final int FLANN_DIST_L1 = 2;
    public static final int FLANN_DIST_L2 = 1;
    public static final int FLANN_DIST_MANHATTAN = 2;
    public static final int FLANN_DIST_MAX = 4;
    public static final int FLANN_DIST_MINKOWSKI = 3;
    public static final int FLANN_FLOAT32 = 8;
    public static final int FLANN_FLOAT64 = 9;
    public static final int FLANN_INDEX_AUTOTUNED = 255;
    public static final int FLANN_INDEX_COMPOSITE = 3;
    public static final int FLANN_INDEX_HIERARCHICAL = 5;
    public static final int FLANN_INDEX_KDTREE = 1;
    public static final int FLANN_INDEX_KDTREE_SINGLE = 4;
    public static final int FLANN_INDEX_KMEANS = 2;
    public static final int FLANN_INDEX_LINEAR = 0;
    public static final int FLANN_INDEX_LSH = 6;
    public static final int FLANN_INDEX_SAVED = 254;
    public static final int FLANN_INDEX_TYPE_16S = 3;
    public static final int FLANN_INDEX_TYPE_16U = 2;
    public static final int FLANN_INDEX_TYPE_32F = 5;
    public static final int FLANN_INDEX_TYPE_32S = 4;
    public static final int FLANN_INDEX_TYPE_64F = 6;
    public static final int FLANN_INDEX_TYPE_8S = 1;
    public static final int FLANN_INDEX_TYPE_8U = 0;
    public static final int FLANN_INDEX_TYPE_ALGORITHM = 9;
    public static final int FLANN_INDEX_TYPE_BOOL = 8;
    public static final int FLANN_INDEX_TYPE_STRING = 7;
    public static final int FLANN_INT16 = 1;
    public static final int FLANN_INT32 = 2;
    public static final int FLANN_INT64 = 3;
    public static final int FLANN_INT8 = 0;
    public static final int FLANN_LOG_ERROR = 2;
    public static final int FLANN_LOG_FATAL = 1;
    public static final int FLANN_LOG_INFO = 4;
    public static final int FLANN_LOG_NONE = 0;
    public static final int FLANN_LOG_WARN = 3;
    public static final int FLANN_UINT16 = 5;
    public static final int FLANN_UINT32 = 6;
    public static final int FLANN_UINT64 = 7;
    public static final int FLANN_UINT8 = 4;
    public static final int HELLINGER = 6;
    public static final int HIST_INTERSECT = 5;
    public static final int KDTREE = 1;
    public static final int KDTREE_SINGLE = 4;
    public static final int KL = 8;
    public static final int KMEANS = 2;
    public static final int KULLBACK_LEIBLER = 8;
    public static final int LAST_VALUE_FLANN_INDEX_TYPE = 9;
    public static final int LINEAR = 0;
    public static final int MANHATTAN = 2;
    public static final int MAX_DIST = 4;
    public static final int MINKOWSKI = 3;
    public static final int SAVED = 254;

    static {
        Loader.load();
    }

    @Namespace("cv::flann")
    @NoOffset
    public static class IndexParams extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        public native void getAll(@ByRef opencv_core.StringVector stringVector, @Cast({"cv::flann::FlannIndexType*"}) @StdVector IntBuffer intBuffer, @ByRef opencv_core.StringVector stringVector2, @StdVector DoubleBuffer doubleBuffer);

        public native void getAll(@ByRef opencv_core.StringVector stringVector, @Cast({"cv::flann::FlannIndexType*"}) @StdVector IntPointer intPointer, @ByRef opencv_core.StringVector stringVector2, @StdVector DoublePointer doublePointer);

        public native void getAll(@ByRef opencv_core.StringVector stringVector, @Cast({"cv::flann::FlannIndexType*"}) @StdVector int[] iArr, @ByRef opencv_core.StringVector stringVector2, @StdVector double[] dArr);

        public native double getDouble(@opencv_core.Str String str);

        public native double getDouble(@opencv_core.Str String str, double d);

        public native double getDouble(@opencv_core.Str BytePointer bytePointer);

        public native double getDouble(@opencv_core.Str BytePointer bytePointer, double d);

        public native int getInt(@opencv_core.Str String str);

        public native int getInt(@opencv_core.Str String str, int i);

        public native int getInt(@opencv_core.Str BytePointer bytePointer);

        public native int getInt(@opencv_core.Str BytePointer bytePointer, int i);

        @opencv_core.Str
        public native String getString(@opencv_core.Str String str);

        @opencv_core.Str
        public native String getString(@opencv_core.Str String str, @opencv_core.Str String str2);

        @opencv_core.Str
        public native BytePointer getString(@opencv_core.Str BytePointer bytePointer);

        @opencv_core.Str
        public native BytePointer getString(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

        public native Pointer params();

        public native IndexParams params(Pointer pointer);

        public native void setAlgorithm(int i);

        public native void setBool(@opencv_core.Str String str, @Cast({"bool"}) boolean z);

        public native void setBool(@opencv_core.Str BytePointer bytePointer, @Cast({"bool"}) boolean z);

        public native void setDouble(@opencv_core.Str String str, double d);

        public native void setDouble(@opencv_core.Str BytePointer bytePointer, double d);

        public native void setFloat(@opencv_core.Str String str, float f);

        public native void setFloat(@opencv_core.Str BytePointer bytePointer, float f);

        public native void setInt(@opencv_core.Str String str, int i);

        public native void setInt(@opencv_core.Str BytePointer bytePointer, int i);

        public native void setString(@opencv_core.Str String str, @opencv_core.Str String str2);

        public native void setString(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

        static {
            Loader.load();
        }

        public IndexParams(Pointer p) {
            super(p);
        }

        public IndexParams(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public IndexParams position(long position) {
            return (IndexParams) super.position(position);
        }

        public IndexParams() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::flann")
    public static class KDTreeIndexParams extends IndexParams {
        private native void allocate();

        private native void allocate(int i);

        private native void allocateArray(long j);

        static {
            Loader.load();
        }

        public KDTreeIndexParams(Pointer p) {
            super(p);
        }

        public KDTreeIndexParams(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public KDTreeIndexParams position(long position) {
            return (KDTreeIndexParams) super.position(position);
        }

        public KDTreeIndexParams(int trees) {
            super((Pointer) null);
            allocate(trees);
        }

        public KDTreeIndexParams() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::flann")
    public static class LinearIndexParams extends IndexParams {
        private native void allocate();

        private native void allocateArray(long j);

        static {
            Loader.load();
        }

        public LinearIndexParams(Pointer p) {
            super(p);
        }

        public LinearIndexParams(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public LinearIndexParams position(long position) {
            return (LinearIndexParams) super.position(position);
        }

        public LinearIndexParams() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::flann")
    public static class CompositeIndexParams extends IndexParams {
        private native void allocate();

        private native void allocate(int i, int i2, int i3, @Cast({"cvflann::flann_centers_init_t"}) int i4, float f);

        private native void allocateArray(long j);

        static {
            Loader.load();
        }

        public CompositeIndexParams(Pointer p) {
            super(p);
        }

        public CompositeIndexParams(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CompositeIndexParams position(long position) {
            return (CompositeIndexParams) super.position(position);
        }

        public CompositeIndexParams(int trees, int branching, int iterations, @Cast({"cvflann::flann_centers_init_t"}) int centers_init, float cb_index) {
            super((Pointer) null);
            allocate(trees, branching, iterations, centers_init, cb_index);
        }

        public CompositeIndexParams() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::flann")
    public static class AutotunedIndexParams extends IndexParams {
        private native void allocate();

        private native void allocate(float f, float f2, float f3, float f4);

        private native void allocateArray(long j);

        static {
            Loader.load();
        }

        public AutotunedIndexParams(Pointer p) {
            super(p);
        }

        public AutotunedIndexParams(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public AutotunedIndexParams position(long position) {
            return (AutotunedIndexParams) super.position(position);
        }

        public AutotunedIndexParams(float target_precision, float build_weight, float memory_weight, float sample_fraction) {
            super((Pointer) null);
            allocate(target_precision, build_weight, memory_weight, sample_fraction);
        }

        public AutotunedIndexParams() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::flann")
    public static class HierarchicalClusteringIndexParams extends IndexParams {
        private native void allocate();

        private native void allocate(int i, @Cast({"cvflann::flann_centers_init_t"}) int i2, int i3, int i4);

        private native void allocateArray(long j);

        static {
            Loader.load();
        }

        public HierarchicalClusteringIndexParams(Pointer p) {
            super(p);
        }

        public HierarchicalClusteringIndexParams(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public HierarchicalClusteringIndexParams position(long position) {
            return (HierarchicalClusteringIndexParams) super.position(position);
        }

        public HierarchicalClusteringIndexParams(int branching, @Cast({"cvflann::flann_centers_init_t"}) int centers_init, int trees, int leaf_size) {
            super((Pointer) null);
            allocate(branching, centers_init, trees, leaf_size);
        }

        public HierarchicalClusteringIndexParams() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::flann")
    public static class KMeansIndexParams extends IndexParams {
        private native void allocate();

        private native void allocate(int i, int i2, @Cast({"cvflann::flann_centers_init_t"}) int i3, float f);

        private native void allocateArray(long j);

        static {
            Loader.load();
        }

        public KMeansIndexParams(Pointer p) {
            super(p);
        }

        public KMeansIndexParams(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public KMeansIndexParams position(long position) {
            return (KMeansIndexParams) super.position(position);
        }

        public KMeansIndexParams(int branching, int iterations, @Cast({"cvflann::flann_centers_init_t"}) int centers_init, float cb_index) {
            super((Pointer) null);
            allocate(branching, iterations, centers_init, cb_index);
        }

        public KMeansIndexParams() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::flann")
    public static class LshIndexParams extends IndexParams {
        private native void allocate(int i, int i2, int i3);

        static {
            Loader.load();
        }

        public LshIndexParams(Pointer p) {
            super(p);
        }

        public LshIndexParams(int table_number, int key_size, int multi_probe_level) {
            super((Pointer) null);
            allocate(table_number, key_size, multi_probe_level);
        }
    }

    @Namespace("cv::flann")
    public static class SavedIndexParams extends IndexParams {
        private native void allocate(@opencv_core.Str String str);

        private native void allocate(@opencv_core.Str BytePointer bytePointer);

        static {
            Loader.load();
        }

        public SavedIndexParams(Pointer p) {
            super(p);
        }

        public SavedIndexParams(@opencv_core.Str BytePointer filename) {
            super((Pointer) null);
            allocate(filename);
        }

        public SavedIndexParams(@opencv_core.Str String filename) {
            super((Pointer) null);
            allocate(filename);
        }
    }

    @Namespace("cv::flann")
    public static class SearchParams extends IndexParams {
        private native void allocate();

        private native void allocate(int i, float f, @Cast({"bool"}) boolean z);

        private native void allocateArray(long j);

        static {
            Loader.load();
        }

        public SearchParams(Pointer p) {
            super(p);
        }

        public SearchParams(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public SearchParams position(long position) {
            return (SearchParams) super.position(position);
        }

        public SearchParams(int checks, float eps, @Cast({"bool"}) boolean sorted) {
            super((Pointer) null);
            allocate(checks, eps, sorted);
        }

        public SearchParams() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::flann")
    @NoOffset
    public static class Index extends Pointer {
        private native void allocate();

        private native void allocate(@ByVal opencv_core.GpuMat gpuMat, @ByRef @Const IndexParams indexParams);

        private native void allocate(@ByVal opencv_core.GpuMat gpuMat, @ByRef @Const IndexParams indexParams, @Cast({"cvflann::flann_distance_t"}) int i);

        private native void allocate(@ByVal opencv_core.Mat mat, @ByRef @Const IndexParams indexParams);

        private native void allocate(@ByVal opencv_core.Mat mat, @ByRef @Const IndexParams indexParams, @Cast({"cvflann::flann_distance_t"}) int i);

        private native void allocate(@ByVal opencv_core.UMat uMat, @ByRef @Const IndexParams indexParams);

        private native void allocate(@ByVal opencv_core.UMat uMat, @ByRef @Const IndexParams indexParams, @Cast({"cvflann::flann_distance_t"}) int i);

        private native void allocateArray(long j);

        public native void build(@ByVal opencv_core.GpuMat gpuMat, @ByRef @Const IndexParams indexParams);

        public native void build(@ByVal opencv_core.GpuMat gpuMat, @ByRef @Const IndexParams indexParams, @Cast({"cvflann::flann_distance_t"}) int i);

        public native void build(@ByVal opencv_core.Mat mat, @ByRef @Const IndexParams indexParams);

        public native void build(@ByVal opencv_core.Mat mat, @ByRef @Const IndexParams indexParams, @Cast({"cvflann::flann_distance_t"}) int i);

        public native void build(@ByVal opencv_core.UMat uMat, @ByRef @Const IndexParams indexParams);

        public native void build(@ByVal opencv_core.UMat uMat, @ByRef @Const IndexParams indexParams, @Cast({"cvflann::flann_distance_t"}) int i);

        @Cast({"cvflann::flann_algorithm_t"})
        public native int getAlgorithm();

        @Cast({"cvflann::flann_distance_t"})
        public native int getDistance();

        public native void knnSearch(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, int i);

        public native void knnSearch(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, int i, @ByRef(nullValue = "cv::flann::SearchParams()") @Const SearchParams searchParams);

        public native void knnSearch(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, int i);

        public native void knnSearch(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, int i, @ByRef(nullValue = "cv::flann::SearchParams()") @Const SearchParams searchParams);

        public native void knnSearch(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, int i);

        public native void knnSearch(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, int i, @ByRef(nullValue = "cv::flann::SearchParams()") @Const SearchParams searchParams);

        @Cast({"bool"})
        public native boolean load(@ByVal opencv_core.GpuMat gpuMat, @opencv_core.Str String str);

        @Cast({"bool"})
        public native boolean load(@ByVal opencv_core.GpuMat gpuMat, @opencv_core.Str BytePointer bytePointer);

        @Cast({"bool"})
        public native boolean load(@ByVal opencv_core.Mat mat, @opencv_core.Str String str);

        @Cast({"bool"})
        public native boolean load(@ByVal opencv_core.Mat mat, @opencv_core.Str BytePointer bytePointer);

        @Cast({"bool"})
        public native boolean load(@ByVal opencv_core.UMat uMat, @opencv_core.Str String str);

        @Cast({"bool"})
        public native boolean load(@ByVal opencv_core.UMat uMat, @opencv_core.Str BytePointer bytePointer);

        public native int radiusSearch(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, double d, int i);

        public native int radiusSearch(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, double d, int i, @ByRef(nullValue = "cv::flann::SearchParams()") @Const SearchParams searchParams);

        public native int radiusSearch(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, double d, int i);

        public native int radiusSearch(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, double d, int i, @ByRef(nullValue = "cv::flann::SearchParams()") @Const SearchParams searchParams);

        public native int radiusSearch(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, double d, int i);

        public native int radiusSearch(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, double d, int i, @ByRef(nullValue = "cv::flann::SearchParams()") @Const SearchParams searchParams);

        public native void release();

        public native void save(@opencv_core.Str String str);

        public native void save(@opencv_core.Str BytePointer bytePointer);

        static {
            Loader.load();
        }

        public Index(Pointer p) {
            super(p);
        }

        public Index(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Index position(long position) {
            return (Index) super.position(position);
        }

        public Index() {
            super((Pointer) null);
            allocate();
        }

        public Index(@ByVal opencv_core.Mat features, @ByRef @Const IndexParams params, @Cast({"cvflann::flann_distance_t"}) int distType) {
            super((Pointer) null);
            allocate(features, params, distType);
        }

        public Index(@ByVal opencv_core.Mat features, @ByRef @Const IndexParams params) {
            super((Pointer) null);
            allocate(features, params);
        }

        public Index(@ByVal opencv_core.UMat features, @ByRef @Const IndexParams params, @Cast({"cvflann::flann_distance_t"}) int distType) {
            super((Pointer) null);
            allocate(features, params, distType);
        }

        public Index(@ByVal opencv_core.UMat features, @ByRef @Const IndexParams params) {
            super((Pointer) null);
            allocate(features, params);
        }

        public Index(@ByVal opencv_core.GpuMat features, @ByRef @Const IndexParams params, @Cast({"cvflann::flann_distance_t"}) int distType) {
            super((Pointer) null);
            allocate(features, params, distType);
        }

        public Index(@ByVal opencv_core.GpuMat features, @ByRef @Const IndexParams params) {
            super((Pointer) null);
            allocate(features, params);
        }
    }
}
