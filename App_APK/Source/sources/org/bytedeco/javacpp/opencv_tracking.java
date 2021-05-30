package org.bytedeco.javacpp;

import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.Arrays;
import org.bytedeco.javacpp.annotation.ByPtrPtr;
import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Const;
import org.bytedeco.javacpp.annotation.Function;
import org.bytedeco.javacpp.annotation.Index;
import org.bytedeco.javacpp.annotation.MemberSetter;
import org.bytedeco.javacpp.annotation.Name;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.annotation.NoOffset;
import org.bytedeco.javacpp.annotation.StdString;
import org.bytedeco.javacpp.annotation.StdVector;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.presets.opencv_core;
import org.xbill.DNS.TTL;

public class opencv_tracking extends org.bytedeco.javacpp.presets.opencv_tracking {
    public static final String CC_FEATURES = "features";
    public static final String CC_FEATURE_PARAMS = "featureParams";
    public static final String CC_FEATURE_SIZE = "featSize";
    public static final String CC_ISINTEGRAL = "isIntegral";
    public static final String CC_MAX_CAT_COUNT = "maxCatCount";
    public static final String CC_NUM_FEATURES = "numFeat";
    public static final String CC_RECT = "rect";
    public static final String CC_RECTS = "rects";
    public static final String CC_TILTED = "tilted";
    public static final int CV_HAAR_FEATURE_MAX = 3;
    public static final String FEATURES = "features";
    public static final String HFP_NAME = "haarFeatureParams";
    public static final String HOGF_NAME = "HOGFeatureParams";
    public static final String LBPF_NAME = "lbpFeatureParams";
    public static final int N_BINS = 9;
    public static final int N_CELLS = 4;

    @Namespace("cv::tracking")
    @opencv_core.Ptr
    public static native UnscentedKalmanFilter createAugmentedUnscentedKalmanFilter(@ByRef @Const AugmentedUnscentedKalmanFilterParams augmentedUnscentedKalmanFilterParams);

    @Namespace("cv::tracking")
    @opencv_core.Ptr
    public static native UnscentedKalmanFilter createUnscentedKalmanFilter(@ByRef @Const UnscentedKalmanFilterParams unscentedKalmanFilterParams);

    @Namespace("cv::tld")
    @ByVal
    public static native opencv_core.Rect2d tld_InitDataset(int i);

    @Namespace("cv::tld")
    @ByVal
    public static native opencv_core.Rect2d tld_InitDataset(int i, String str, int i2);

    @Namespace("cv::tld")
    @ByVal
    public static native opencv_core.Rect2d tld_InitDataset(int i, @Cast({"const char*"}) BytePointer bytePointer, int i2);

    @Namespace("cv::tld")
    @opencv_core.Str
    public static native BytePointer tld_getNextDatasetFrame();

    static {
        Loader.load();
    }

    @Name({"std::vector<cv::Ptr<cv::Tracker> >"})
    public static class TrackerVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @Index(function = "at")
        @opencv_core.Ptr
        public native Tracker get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @opencv_core.Ptr Tracker tracker);

        public native TrackerVector put(@Cast({"size_t"}) long j, Tracker tracker);

        @ByRef
        @Name({"operator="})
        public native TrackerVector put(@ByRef TrackerVector trackerVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public TrackerVector(Pointer p) {
            super(p);
        }

        public TrackerVector(Tracker value) {
            this(1);
            put(0, value);
        }

        public TrackerVector(Tracker... array) {
            this((long) array.length);
            put(array);
        }

        public TrackerVector() {
            allocate();
        }

        public TrackerVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @opencv_core.Ptr
            @Const
            @Name({"operator*"})
            public native Tracker get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public Tracker[] get() {
            Tracker[] array = new Tracker[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return Arrays.toString(get());
        }

        public Tracker pop_back() {
            long size = size();
            Tracker value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public TrackerVector push_back(Tracker value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public TrackerVector put(Tracker value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public TrackerVector put(Tracker... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<cv::ConfidenceMap>"})
    public static class ConfidenceMapVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @ByRef
        @Index(function = "at")
        public native ConfidenceMap get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @ByRef ConfidenceMap confidenceMap);

        public native ConfidenceMapVector put(@Cast({"size_t"}) long j, ConfidenceMap confidenceMap);

        @ByRef
        @Name({"operator="})
        public native ConfidenceMapVector put(@ByRef ConfidenceMapVector confidenceMapVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public ConfidenceMapVector(Pointer p) {
            super(p);
        }

        public ConfidenceMapVector(ConfidenceMap value) {
            this(1);
            put(0, value);
        }

        public ConfidenceMapVector(ConfidenceMap... array) {
            this((long) array.length);
            put(array);
        }

        public ConfidenceMapVector() {
            allocate();
        }

        public ConfidenceMapVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @ByRef
            @Const
            @Name({"operator*"})
            public native ConfidenceMap get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public ConfidenceMap[] get() {
            ConfidenceMap[] array = new ConfidenceMap[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return Arrays.toString(get());
        }

        public ConfidenceMap pop_back() {
            long size = size();
            ConfidenceMap value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public ConfidenceMapVector push_back(ConfidenceMap value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public ConfidenceMapVector put(ConfidenceMap value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public ConfidenceMapVector put(ConfidenceMap... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<std::pair<cv::Ptr<cv::TrackerTargetState>,float> >"})
    public static class ConfidenceMap extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        public native ConfidenceMap first(@Cast({"size_t"}) long j, TrackerTargetState trackerTargetState);

        @Index(function = "at")
        @opencv_core.Ptr
        public native TrackerTargetState first(@Cast({"size_t"}) long j);

        @ByRef
        @Name({"operator="})
        public native ConfidenceMap put(@ByRef ConfidenceMap confidenceMap);

        public native void resize(@Cast({"size_t"}) long j);

        @Index(function = "at")
        public native float second(@Cast({"size_t"}) long j);

        public native ConfidenceMap second(@Cast({"size_t"}) long j, float f);

        public native long size();

        static {
            Loader.load();
        }

        public ConfidenceMap(Pointer p) {
            super(p);
        }

        public ConfidenceMap(TrackerTargetState[] firstValue, float[] secondValue) {
            this((long) Math.min(firstValue.length, secondValue.length));
            put(firstValue, secondValue);
        }

        public ConfidenceMap() {
            allocate();
        }

        public ConfidenceMap(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        public ConfidenceMap put(TrackerTargetState[] firstValue, float[] secondValue) {
            int i = 0;
            while (i < firstValue.length && i < secondValue.length) {
                first((long) i, firstValue[i]);
                second((long) i, secondValue[i]);
                i++;
            }
            return this;
        }
    }

    @Name({"std::vector<std::pair<cv::String,cv::Ptr<cv::TrackerFeature> > >"})
    public static class StringTrackerFeaturePairVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @opencv_core.Str
        @Index(function = "at")
        public native BytePointer first(@Cast({"size_t"}) long j);

        @Index(function = "at")
        @MemberSetter
        public native StringTrackerFeaturePairVector first(@Cast({"size_t"}) long j, @opencv_core.Str String str);

        public native StringTrackerFeaturePairVector first(@Cast({"size_t"}) long j, BytePointer bytePointer);

        @ByRef
        @Name({"operator="})
        public native StringTrackerFeaturePairVector put(@ByRef StringTrackerFeaturePairVector stringTrackerFeaturePairVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native StringTrackerFeaturePairVector second(@Cast({"size_t"}) long j, TrackerFeature trackerFeature);

        @Index(function = "at")
        @opencv_core.Ptr
        public native TrackerFeature second(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public StringTrackerFeaturePairVector(Pointer p) {
            super(p);
        }

        public StringTrackerFeaturePairVector(BytePointer[] firstValue, TrackerFeature[] secondValue) {
            this((long) Math.min(firstValue.length, secondValue.length));
            put(firstValue, secondValue);
        }

        public StringTrackerFeaturePairVector(String[] firstValue, TrackerFeature[] secondValue) {
            this((long) Math.min(firstValue.length, secondValue.length));
            put(firstValue, secondValue);
        }

        public StringTrackerFeaturePairVector() {
            allocate();
        }

        public StringTrackerFeaturePairVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        public StringTrackerFeaturePairVector put(BytePointer[] firstValue, TrackerFeature[] secondValue) {
            int i = 0;
            while (i < firstValue.length && i < secondValue.length) {
                first((long) i, firstValue[i]);
                second((long) i, secondValue[i]);
                i++;
            }
            return this;
        }

        public StringTrackerFeaturePairVector put(String[] firstValue, TrackerFeature[] secondValue) {
            int i = 0;
            while (i < firstValue.length && i < secondValue.length) {
                first((long) i, firstValue[i]);
                second((long) i, secondValue[i]);
                i++;
            }
            return this;
        }
    }

    @Name({"std::vector<std::pair<cv::String,cv::Ptr<cv::TrackerSamplerAlgorithm> > >"})
    public static class StringTrackerSamplerAlgorithmPairVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @opencv_core.Str
        @Index(function = "at")
        public native BytePointer first(@Cast({"size_t"}) long j);

        @Index(function = "at")
        @MemberSetter
        public native StringTrackerSamplerAlgorithmPairVector first(@Cast({"size_t"}) long j, @opencv_core.Str String str);

        public native StringTrackerSamplerAlgorithmPairVector first(@Cast({"size_t"}) long j, BytePointer bytePointer);

        @ByRef
        @Name({"operator="})
        public native StringTrackerSamplerAlgorithmPairVector put(@ByRef StringTrackerSamplerAlgorithmPairVector stringTrackerSamplerAlgorithmPairVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native StringTrackerSamplerAlgorithmPairVector second(@Cast({"size_t"}) long j, TrackerSamplerAlgorithm trackerSamplerAlgorithm);

        @Index(function = "at")
        @opencv_core.Ptr
        public native TrackerSamplerAlgorithm second(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public StringTrackerSamplerAlgorithmPairVector(Pointer p) {
            super(p);
        }

        public StringTrackerSamplerAlgorithmPairVector(BytePointer[] firstValue, TrackerSamplerAlgorithm[] secondValue) {
            this((long) Math.min(firstValue.length, secondValue.length));
            put(firstValue, secondValue);
        }

        public StringTrackerSamplerAlgorithmPairVector(String[] firstValue, TrackerSamplerAlgorithm[] secondValue) {
            this((long) Math.min(firstValue.length, secondValue.length));
            put(firstValue, secondValue);
        }

        public StringTrackerSamplerAlgorithmPairVector() {
            allocate();
        }

        public StringTrackerSamplerAlgorithmPairVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        public StringTrackerSamplerAlgorithmPairVector put(BytePointer[] firstValue, TrackerSamplerAlgorithm[] secondValue) {
            int i = 0;
            while (i < firstValue.length && i < secondValue.length) {
                first((long) i, firstValue[i]);
                second((long) i, secondValue[i]);
                i++;
            }
            return this;
        }

        public StringTrackerSamplerAlgorithmPairVector put(String[] firstValue, TrackerSamplerAlgorithm[] secondValue) {
            int i = 0;
            while (i < firstValue.length && i < secondValue.length) {
                first((long) i, firstValue[i]);
                second((long) i, secondValue[i]);
                i++;
            }
            return this;
        }
    }

    @Name({"std::vector<cv::Ptr<cv::TrackerTargetState> >"})
    public static class Trajectory extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @Index(function = "at")
        @opencv_core.Ptr
        public native TrackerTargetState get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @opencv_core.Ptr TrackerTargetState trackerTargetState);

        public native Trajectory put(@Cast({"size_t"}) long j, TrackerTargetState trackerTargetState);

        @ByRef
        @Name({"operator="})
        public native Trajectory put(@ByRef Trajectory trajectory);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public Trajectory(Pointer p) {
            super(p);
        }

        public Trajectory(TrackerTargetState value) {
            this(1);
            put(0, value);
        }

        public Trajectory(TrackerTargetState... array) {
            this((long) array.length);
            put(array);
        }

        public Trajectory() {
            allocate();
        }

        public Trajectory(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @opencv_core.Ptr
            @Const
            @Name({"operator*"})
            public native TrackerTargetState get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public TrackerTargetState[] get() {
            TrackerTargetState[] array = new TrackerTargetState[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return Arrays.toString(get());
        }

        public TrackerTargetState pop_back() {
            long size = size();
            TrackerTargetState value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public Trajectory push_back(TrackerTargetState value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public Trajectory put(TrackerTargetState value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public Trajectory put(TrackerTargetState... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class CvParams extends Pointer {
        @StdString
        public native BytePointer name();

        public native CvParams name(BytePointer bytePointer);

        public native void printAttrs();

        public native void printDefaults();

        @Cast({"bool"})
        public native boolean read(@ByRef @Const opencv_core.FileNode fileNode);

        @Cast({"bool"})
        public native boolean scanAttr(@StdString String str, @StdString String str2);

        @Cast({"bool"})
        public native boolean scanAttr(@StdString BytePointer bytePointer, @StdString BytePointer bytePointer2);

        public native void write(@ByRef opencv_core.FileStorage fileStorage);

        static {
            Loader.load();
        }

        public CvParams(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class CvFeatureParams extends CvParams {
        public static final int HAAR = 0;
        public static final int HOG = 2;
        public static final int LBP = 1;

        private native void allocate();

        private native void allocateArray(long j);

        @opencv_core.Ptr
        public static native CvFeatureParams create(@Cast({"cv::CvFeatureParams::FeatureType"}) int i);

        public native int featSize();

        public native CvFeatureParams featSize(int i);

        public native void init(@ByRef @Const CvFeatureParams cvFeatureParams);

        public native int maxCatCount();

        public native CvFeatureParams maxCatCount(int i);

        public native int numFeatures();

        public native CvFeatureParams numFeatures(int i);

        @Cast({"bool"})
        public native boolean read(@ByRef @Const opencv_core.FileNode fileNode);

        public native void write(@ByRef opencv_core.FileStorage fileStorage);

        static {
            Loader.load();
        }

        public CvFeatureParams(Pointer p) {
            super(p);
        }

        public CvFeatureParams(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvFeatureParams position(long position) {
            return (CvFeatureParams) super.position(position);
        }

        public CvFeatureParams() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv")
    public static class CvFeatureEvaluator extends Pointer {
        @opencv_core.Ptr
        public static native CvFeatureEvaluator create(@Cast({"cv::CvFeatureParams::FeatureType"}) int i);

        @Name({"operator ()"})
        public native float apply(int i, int i2);

        public native float getCls(int i);

        @ByRef
        @Const
        public native opencv_core.Mat getCls();

        public native int getFeatureSize();

        public native int getMaxCatCount();

        public native int getNumFeatures();

        public native void init(@Const CvFeatureParams cvFeatureParams, int i, @ByVal opencv_core.Size size);

        public native void setImage(@ByRef @Const opencv_core.Mat mat, @Cast({"uchar"}) byte b, int i);

        public native void writeFeatures(@ByRef opencv_core.FileStorage fileStorage, @ByRef @Const opencv_core.Mat mat);

        static {
            Loader.load();
        }

        public CvFeatureEvaluator(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class CvHaarFeatureParams extends CvFeatureParams {
        private native void allocate();

        private native void allocateArray(long j);

        public native void init(@ByRef @Const CvFeatureParams cvFeatureParams);

        public native CvHaarFeatureParams isIntegral(boolean z);

        @Cast({"bool"})
        public native boolean isIntegral();

        public native void printAttrs();

        public native void printDefaults();

        @Cast({"bool"})
        public native boolean read(@ByRef @Const opencv_core.FileNode fileNode);

        @Cast({"bool"})
        public native boolean scanAttr(@StdString String str, @StdString String str2);

        @Cast({"bool"})
        public native boolean scanAttr(@StdString BytePointer bytePointer, @StdString BytePointer bytePointer2);

        public native void write(@ByRef opencv_core.FileStorage fileStorage);

        static {
            Loader.load();
        }

        public CvHaarFeatureParams(Pointer p) {
            super(p);
        }

        public CvHaarFeatureParams(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvHaarFeatureParams position(long position) {
            return (CvHaarFeatureParams) super.position(position);
        }

        public CvHaarFeatureParams() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class CvHaarEvaluator extends CvFeatureEvaluator {
        private native void allocate();

        private native void allocateArray(long j);

        @Name({"operator ()"})
        public native float apply(int i, int i2);

        public native void generateFeatures();

        public native void generateFeatures(int i);

        @StdVector
        public native FeatureHaar getFeatures();

        @ByRef
        public native FeatureHaar getFeatures(int i);

        public native void init(@Const CvFeatureParams cvFeatureParams, int i, @ByVal opencv_core.Size size);

        public native void setImage(@ByRef @Const opencv_core.Mat mat);

        public native void setImage(@ByRef @Const opencv_core.Mat mat, @Cast({"uchar"}) byte b, int i);

        @Function
        @ByVal
        public native opencv_core.Size setWinSize();

        @Function
        public native void setWinSize(@ByVal opencv_core.Size size);

        public native void writeFeature(@ByRef opencv_core.FileStorage fileStorage);

        public native void writeFeatures(@ByRef opencv_core.FileStorage fileStorage, @ByRef @Const opencv_core.Mat mat);

        static {
            Loader.load();
        }

        public CvHaarEvaluator() {
            super((Pointer) null);
            allocate();
        }

        public CvHaarEvaluator(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvHaarEvaluator(Pointer p) {
            super(p);
        }

        public CvHaarEvaluator position(long position) {
            return (CvHaarEvaluator) super.position(position);
        }

        @NoOffset
        public static class FeatureHaar extends Pointer {
            private native void allocate(@ByVal opencv_core.Size size);

            @Cast({"bool"})
            public native boolean eval(@ByRef @Const opencv_core.Mat mat, @ByVal opencv_core.Rect rect, FloatBuffer floatBuffer);

            @Cast({"bool"})
            public native boolean eval(@ByRef @Const opencv_core.Mat mat, @ByVal opencv_core.Rect rect, FloatPointer floatPointer);

            @Cast({"bool"})
            public native boolean eval(@ByRef @Const opencv_core.Mat mat, @ByVal opencv_core.Rect rect, float[] fArr);

            @ByRef
            @Const
            public native opencv_core.RectVector getAreas();

            public native float getInitMean();

            public native float getInitSigma();

            public native int getNumAreas();

            @StdVector
            public native FloatPointer getWeights();

            public native void write(@ByVal opencv_core.FileStorage fileStorage);

            static {
                Loader.load();
            }

            public FeatureHaar(Pointer p) {
                super(p);
            }

            public FeatureHaar(@ByVal opencv_core.Size patchSize) {
                super((Pointer) null);
                allocate(patchSize);
            }
        }
    }

    @Namespace("cv")
    public static class CvHOGFeatureParams extends CvFeatureParams {
        private native void allocate();

        private native void allocateArray(long j);

        static {
            Loader.load();
        }

        public CvHOGFeatureParams(Pointer p) {
            super(p);
        }

        public CvHOGFeatureParams(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvHOGFeatureParams position(long position) {
            return (CvHOGFeatureParams) super.position(position);
        }

        public CvHOGFeatureParams() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class CvHOGEvaluator extends CvFeatureEvaluator {
        private native void allocate();

        private native void allocateArray(long j);

        @Name({"operator ()"})
        public native float apply(int i, int i2);

        public native void init(@Const CvFeatureParams cvFeatureParams, int i, @ByVal opencv_core.Size size);

        public native void setImage(@ByRef @Const opencv_core.Mat mat, @Cast({"uchar"}) byte b, int i);

        public native void writeFeatures(@ByRef opencv_core.FileStorage fileStorage, @ByRef @Const opencv_core.Mat mat);

        static {
            Loader.load();
        }

        public CvHOGEvaluator() {
            super((Pointer) null);
            allocate();
        }

        public CvHOGEvaluator(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvHOGEvaluator(Pointer p) {
            super(p);
        }

        public CvHOGEvaluator position(long position) {
            return (CvHOGEvaluator) super.position(position);
        }
    }

    @Namespace("cv")
    public static class CvLBPFeatureParams extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        static {
            Loader.load();
        }

        public CvLBPFeatureParams(Pointer p) {
            super(p);
        }

        public CvLBPFeatureParams(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvLBPFeatureParams position(long position) {
            return (CvLBPFeatureParams) super.position(position);
        }

        public CvLBPFeatureParams() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class CvLBPEvaluator extends CvFeatureEvaluator {
        private native void allocate();

        private native void allocateArray(long j);

        @Name({"operator ()"})
        public native float apply(int i, int i2);

        public native void init(@Const CvFeatureParams cvFeatureParams, int i, @ByVal opencv_core.Size size);

        public native void setImage(@ByRef @Const opencv_core.Mat mat, @Cast({"uchar"}) byte b, int i);

        public native void writeFeatures(@ByRef opencv_core.FileStorage fileStorage, @ByRef @Const opencv_core.Mat mat);

        static {
            Loader.load();
        }

        public CvLBPEvaluator() {
            super((Pointer) null);
            allocate();
        }

        public CvLBPEvaluator(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvLBPEvaluator(Pointer p) {
            super(p);
        }

        public CvLBPEvaluator position(long position) {
            return (CvLBPEvaluator) super.position(position);
        }
    }

    @Namespace("cv::tracking")
    public static class UnscentedKalmanFilter extends Pointer {
        @ByVal
        public native opencv_core.Mat correct(@ByVal opencv_core.GpuMat gpuMat);

        @ByVal
        public native opencv_core.Mat correct(@ByVal opencv_core.Mat mat);

        @ByVal
        public native opencv_core.Mat correct(@ByVal opencv_core.UMat uMat);

        @ByVal
        public native opencv_core.Mat getErrorCov();

        @ByVal
        public native opencv_core.Mat getMeasurementNoiseCov();

        @ByVal
        public native opencv_core.Mat getProcessNoiseCov();

        @ByVal
        public native opencv_core.Mat getState();

        @ByVal
        public native opencv_core.Mat predict();

        @ByVal
        public native opencv_core.Mat predict(@ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat);

        @ByVal
        public native opencv_core.Mat predict(@ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat);

        @ByVal
        public native opencv_core.Mat predict(@ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat);

        static {
            Loader.load();
        }

        public UnscentedKalmanFilter(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::tracking")
    public static class UkfSystemModel extends Pointer {
        public native void measurementFunction(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2, @ByRef opencv_core.Mat mat3);

        public native void stateConversionFunction(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2, @ByRef @Const opencv_core.Mat mat3, @ByRef opencv_core.Mat mat4);

        static {
            Loader.load();
        }

        public UkfSystemModel(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::tracking")
    @NoOffset
    public static class UnscentedKalmanFilterParams extends Pointer {
        private native void allocate();

        private native void allocate(int i, int i2, int i3, double d, double d2, @opencv_core.Ptr UkfSystemModel ukfSystemModel);

        private native void allocate(int i, int i2, int i3, double d, double d2, @opencv_core.Ptr UkfSystemModel ukfSystemModel, int i4);

        private native void allocateArray(long j);

        public native int CP();

        public native UnscentedKalmanFilterParams CP(int i);

        public native int DP();

        public native UnscentedKalmanFilterParams DP(int i);

        public native int MP();

        public native UnscentedKalmanFilterParams MP(int i);

        public native double alpha();

        public native UnscentedKalmanFilterParams alpha(double d);

        public native double beta();

        public native UnscentedKalmanFilterParams beta(double d);

        public native int dataType();

        public native UnscentedKalmanFilterParams dataType(int i);

        @ByRef
        public native opencv_core.Mat errorCovInit();

        public native UnscentedKalmanFilterParams errorCovInit(opencv_core.Mat mat);

        public native void init(int i, int i2, int i3, double d, double d2, @opencv_core.Ptr UkfSystemModel ukfSystemModel);

        public native void init(int i, int i2, int i3, double d, double d2, @opencv_core.Ptr UkfSystemModel ukfSystemModel, int i4);

        public native double k();

        public native UnscentedKalmanFilterParams k(double d);

        @ByRef
        public native opencv_core.Mat measurementNoiseCov();

        public native UnscentedKalmanFilterParams measurementNoiseCov(opencv_core.Mat mat);

        @opencv_core.Ptr
        public native UkfSystemModel model();

        public native UnscentedKalmanFilterParams model(UkfSystemModel ukfSystemModel);

        @ByRef
        public native opencv_core.Mat processNoiseCov();

        public native UnscentedKalmanFilterParams processNoiseCov(opencv_core.Mat mat);

        @ByRef
        public native opencv_core.Mat stateInit();

        public native UnscentedKalmanFilterParams stateInit(opencv_core.Mat mat);

        static {
            Loader.load();
        }

        public UnscentedKalmanFilterParams(Pointer p) {
            super(p);
        }

        public UnscentedKalmanFilterParams(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public UnscentedKalmanFilterParams position(long position) {
            return (UnscentedKalmanFilterParams) super.position(position);
        }

        public UnscentedKalmanFilterParams() {
            super((Pointer) null);
            allocate();
        }

        public UnscentedKalmanFilterParams(int dp, int mp, int cp, double processNoiseCovDiag, double measurementNoiseCovDiag, @opencv_core.Ptr UkfSystemModel dynamicalSystem, int type) {
            super((Pointer) null);
            allocate(dp, mp, cp, processNoiseCovDiag, measurementNoiseCovDiag, dynamicalSystem, type);
        }

        public UnscentedKalmanFilterParams(int dp, int mp, int cp, double processNoiseCovDiag, double measurementNoiseCovDiag, @opencv_core.Ptr UkfSystemModel dynamicalSystem) {
            super((Pointer) null);
            allocate(dp, mp, cp, processNoiseCovDiag, measurementNoiseCovDiag, dynamicalSystem);
        }
    }

    @Namespace("cv::tracking")
    public static class AugmentedUnscentedKalmanFilterParams extends UnscentedKalmanFilterParams {
        private native void allocate();

        private native void allocate(int i, int i2, int i3, double d, double d2, @opencv_core.Ptr UkfSystemModel ukfSystemModel);

        private native void allocate(int i, int i2, int i3, double d, double d2, @opencv_core.Ptr UkfSystemModel ukfSystemModel, int i4);

        private native void allocateArray(long j);

        public native void init(int i, int i2, int i3, double d, double d2, @opencv_core.Ptr UkfSystemModel ukfSystemModel);

        public native void init(int i, int i2, int i3, double d, double d2, @opencv_core.Ptr UkfSystemModel ukfSystemModel, int i4);

        static {
            Loader.load();
        }

        public AugmentedUnscentedKalmanFilterParams(Pointer p) {
            super(p);
        }

        public AugmentedUnscentedKalmanFilterParams(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public AugmentedUnscentedKalmanFilterParams position(long position) {
            return (AugmentedUnscentedKalmanFilterParams) super.position(position);
        }

        public AugmentedUnscentedKalmanFilterParams() {
            super((Pointer) null);
            allocate();
        }

        public AugmentedUnscentedKalmanFilterParams(int dp, int mp, int cp, double processNoiseCovDiag, double measurementNoiseCovDiag, @opencv_core.Ptr UkfSystemModel dynamicalSystem, int type) {
            super((Pointer) null);
            allocate(dp, mp, cp, processNoiseCovDiag, measurementNoiseCovDiag, dynamicalSystem, type);
        }

        public AugmentedUnscentedKalmanFilterParams(int dp, int mp, int cp, double processNoiseCovDiag, double measurementNoiseCovDiag, @opencv_core.Ptr UkfSystemModel dynamicalSystem) {
            super((Pointer) null);
            allocate(dp, mp, cp, processNoiseCovDiag, measurementNoiseCovDiag, dynamicalSystem);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class ClfMilBoost extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        @StdVector
        public native FloatPointer classify(@ByRef @Const opencv_core.Mat mat);

        @StdVector
        public native FloatPointer classify(@ByRef @Const opencv_core.Mat mat, @Cast({"bool"}) boolean z);

        public native void init();

        public native void init(@ByRef(nullValue = "cv::ClfMilBoost::Params()") @Const Params params);

        public native float sigmoid(float f);

        public native void update(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2);

        static {
            Loader.load();
        }

        public ClfMilBoost(Pointer p) {
            super(p);
        }

        public ClfMilBoost(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public ClfMilBoost position(long position) {
            return (ClfMilBoost) super.position(position);
        }

        @NoOffset
        public static class Params extends Pointer {
            private native void allocate();

            private native void allocateArray(long j);

            public native float _lRate();

            public native Params _lRate(float f);

            public native int _numFeat();

            public native Params _numFeat(int i);

            public native int _numSel();

            public native Params _numSel(int i);

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

        public ClfMilBoost() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class ClfOnlineStump extends Pointer {
        private native void allocate();

        private native void allocate(int i);

        private native void allocateArray(long j);

        public native float _e0();

        public native ClfOnlineStump _e0(float f);

        public native float _e1();

        public native ClfOnlineStump _e1(float f);

        public native float _lRate();

        public native ClfOnlineStump _lRate(float f);

        public native float _log_n0();

        public native ClfOnlineStump _log_n0(float f);

        public native float _log_n1();

        public native ClfOnlineStump _log_n1(float f);

        public native float _mu0();

        public native ClfOnlineStump _mu0(float f);

        public native float _mu1();

        public native ClfOnlineStump _mu1(float f);

        public native float _q();

        public native ClfOnlineStump _q(float f);

        public native int _s();

        public native ClfOnlineStump _s(int i);

        public native float _sig0();

        public native ClfOnlineStump _sig0(float f);

        public native float _sig1();

        public native ClfOnlineStump _sig1(float f);

        @Cast({"bool"})
        public native boolean classify(@ByRef @Const opencv_core.Mat mat, int i);

        public native float classifyF(@ByRef @Const opencv_core.Mat mat, int i);

        @StdVector
        public native FloatPointer classifySetF(@ByRef @Const opencv_core.Mat mat);

        public native void init();

        public native void update(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2);

        static {
            Loader.load();
        }

        public ClfOnlineStump(Pointer p) {
            super(p);
        }

        public ClfOnlineStump(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public ClfOnlineStump position(long position) {
            return (ClfOnlineStump) super.position(position);
        }

        public ClfOnlineStump() {
            super((Pointer) null);
            allocate();
        }

        public ClfOnlineStump(int ind) {
            super((Pointer) null);
            allocate(ind);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class StrongClassifierDirectSelection extends Pointer {
        private native void allocate(int i, int i2, @ByVal opencv_core.Size size, @ByRef @Const opencv_core.Rect rect);

        private native void allocate(int i, int i2, @ByVal opencv_core.Size size, @ByRef @Const opencv_core.Rect rect, @Cast({"bool"}) boolean z, int i3);

        public native float classifySmooth(@ByRef @Const opencv_core.MatVector matVector, @ByRef @Const opencv_core.Rect rect, @ByRef IntBuffer intBuffer);

        public native float classifySmooth(@ByRef @Const opencv_core.MatVector matVector, @ByRef @Const opencv_core.Rect rect, @ByRef IntPointer intPointer);

        public native float classifySmooth(@ByRef @Const opencv_core.MatVector matVector, @ByRef @Const opencv_core.Rect rect, @ByRef int[] iArr);

        public native float eval(@ByRef @Const opencv_core.Mat mat);

        public native int getNumBaseClassifier();

        @ByVal
        public native opencv_core.Size getPatchSize();

        @ByVal
        public native opencv_core.Rect getROI();

        public native int getReplacedClassifier();

        @StdVector
        public native IntPointer getSelectedWeakClassifier();

        public native int getSwappedClassifier();

        @Cast({"bool"})
        public native boolean getUseFeatureExchange();

        public native void initBaseClassifier();

        public native void replaceWeakClassifier(int i);

        @Cast({"bool"})
        public native boolean update(@ByRef @Const opencv_core.Mat mat, int i);

        @Cast({"bool"})
        public native boolean update(@ByRef @Const opencv_core.Mat mat, int i, float f);

        static {
            Loader.load();
        }

        public StrongClassifierDirectSelection(Pointer p) {
            super(p);
        }

        public StrongClassifierDirectSelection(int numBaseClf, int numWeakClf, @ByVal opencv_core.Size patchSz, @ByRef @Const opencv_core.Rect sampleROI, @Cast({"bool"}) boolean useFeatureEx, int iterationInit) {
            super((Pointer) null);
            allocate(numBaseClf, numWeakClf, patchSz, sampleROI, useFeatureEx, iterationInit);
        }

        public StrongClassifierDirectSelection(int numBaseClf, int numWeakClf, @ByVal opencv_core.Size patchSz, @ByRef @Const opencv_core.Rect sampleROI) {
            super((Pointer) null);
            allocate(numBaseClf, numWeakClf, patchSz, sampleROI);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class BaseClassifier extends Pointer {
        private native void allocate(int i, int i2);

        private native void allocate(int i, int i2, @Cast({"cv::WeakClassifierHaarFeature**"}) PointerPointer pointerPointer);

        private native void allocate(int i, int i2, @ByPtrPtr WeakClassifierHaarFeature weakClassifierHaarFeature);

        public native int computeReplaceWeakestClassifier(@StdVector FloatBuffer floatBuffer);

        public native int computeReplaceWeakestClassifier(@StdVector FloatPointer floatPointer);

        public native int computeReplaceWeakestClassifier(@StdVector float[] fArr);

        public native int eval(@ByRef @Const opencv_core.Mat mat);

        public native float getError(int i);

        public native void getErrors(FloatBuffer floatBuffer);

        public native void getErrors(FloatPointer floatPointer);

        public native void getErrors(float[] fArr);

        public native int getIdxOfNewWeakClassifier();

        @Cast({"cv::WeakClassifierHaarFeature**"})
        public native PointerPointer getReferenceWeakClassifier();

        public native int getSelectedClassifier();

        public native void replaceClassifierStatistic(int i, int i2);

        public native void replaceWeakClassifier(int i);

        public native int selectBestClassifier(@Cast({"bool*"}) @StdVector BoolPointer boolPointer, float f, @StdVector FloatBuffer floatBuffer);

        public native int selectBestClassifier(@Cast({"bool*"}) @StdVector BoolPointer boolPointer, float f, @StdVector FloatPointer floatPointer);

        public native int selectBestClassifier(@Cast({"bool*"}) @StdVector BoolPointer boolPointer, float f, @StdVector float[] fArr);

        public native int selectBestClassifier(@Cast({"bool*"}) @StdVector boolean[] zArr, float f, @StdVector FloatBuffer floatBuffer);

        public native int selectBestClassifier(@Cast({"bool*"}) @StdVector boolean[] zArr, float f, @StdVector FloatPointer floatPointer);

        public native int selectBestClassifier(@Cast({"bool*"}) @StdVector boolean[] zArr, float f, @StdVector float[] fArr);

        public native void trainClassifier(@ByRef @Const opencv_core.Mat mat, int i, float f, @Cast({"bool*"}) @StdVector BoolPointer boolPointer);

        public native void trainClassifier(@ByRef @Const opencv_core.Mat mat, int i, float f, @Cast({"bool*"}) @StdVector boolean[] zArr);

        static {
            Loader.load();
        }

        public BaseClassifier(Pointer p) {
            super(p);
        }

        public BaseClassifier(int numWeakClassifier, int iterationInit) {
            super((Pointer) null);
            allocate(numWeakClassifier, iterationInit);
        }

        public BaseClassifier(int numWeakClassifier, int iterationInit, @Cast({"cv::WeakClassifierHaarFeature**"}) PointerPointer weakCls) {
            super((Pointer) null);
            allocate(numWeakClassifier, iterationInit, weakCls);
        }

        public BaseClassifier(int numWeakClassifier, int iterationInit, @ByPtrPtr WeakClassifierHaarFeature weakCls) {
            super((Pointer) null);
            allocate(numWeakClassifier, iterationInit, weakCls);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class EstimatedGaussDistribution extends Pointer {
        private native void allocate();

        private native void allocate(float f, float f2, float f3, float f4);

        private native void allocateArray(long j);

        public native float getMean();

        public native float getSigma();

        public native void setValues(float f, float f2);

        public native void update(float f);

        static {
            Loader.load();
        }

        public EstimatedGaussDistribution(Pointer p) {
            super(p);
        }

        public EstimatedGaussDistribution(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public EstimatedGaussDistribution position(long position) {
            return (EstimatedGaussDistribution) super.position(position);
        }

        public EstimatedGaussDistribution() {
            super((Pointer) null);
            allocate();
        }

        public EstimatedGaussDistribution(float P_mean, float R_mean, float P_sigma, float R_sigma) {
            super((Pointer) null);
            allocate(P_mean, R_mean, P_sigma, R_sigma);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class WeakClassifierHaarFeature extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        public native int eval(float f);

        @Cast({"bool"})
        public native boolean update(float f, int i);

        static {
            Loader.load();
        }

        public WeakClassifierHaarFeature(Pointer p) {
            super(p);
        }

        public WeakClassifierHaarFeature(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public WeakClassifierHaarFeature position(long position) {
            return (WeakClassifierHaarFeature) super.position(position);
        }

        public WeakClassifierHaarFeature() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class Detector extends Pointer {
        private native void allocate(StrongClassifierDirectSelection strongClassifierDirectSelection);

        public native void classifySmooth(@ByRef @Const opencv_core.MatVector matVector);

        public native void classifySmooth(@ByRef @Const opencv_core.MatVector matVector, float f);

        @ByRef
        @Const
        public native opencv_core.Mat getConfImageDisplay();

        public native float getConfidence(int i);

        public native float getConfidenceOfBestDetection();

        public native float getConfidenceOfDetection(int i);

        @StdVector
        public native FloatPointer getConfidences();

        @StdVector
        public native IntPointer getIdxDetections();

        public native int getNumDetections();

        public native int getPatchIdxOfBestDetection();

        public native int getPatchIdxOfDetection(int i);

        static {
            Loader.load();
        }

        public Detector(Pointer p) {
            super(p);
        }

        public Detector(StrongClassifierDirectSelection classifier) {
            super((Pointer) null);
            allocate(classifier);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class ClassifierThreshold extends Pointer {
        private native void allocate(EstimatedGaussDistribution estimatedGaussDistribution, EstimatedGaussDistribution estimatedGaussDistribution2);

        public native int eval(float f);

        public native Pointer getDistribution(int i);

        public native void update(float f, int i);

        static {
            Loader.load();
        }

        public ClassifierThreshold(Pointer p) {
            super(p);
        }

        public ClassifierThreshold(EstimatedGaussDistribution posSamples, EstimatedGaussDistribution negSamples) {
            super((Pointer) null);
            allocate(posSamples, negSamples);
        }
    }

    @Namespace("cv")
    public static class TrackerFeature extends Pointer {
        @opencv_core.Ptr
        @ByVal
        public static native TrackerFeature create(@opencv_core.Str String str);

        @opencv_core.Ptr
        @ByVal
        public static native TrackerFeature create(@opencv_core.Str BytePointer bytePointer);

        public native void compute(@ByRef @Const opencv_core.MatVector matVector, @ByRef opencv_core.Mat mat);

        @opencv_core.Str
        public native BytePointer getClassName();

        public native void selection(@ByRef opencv_core.Mat mat, int i);

        static {
            Loader.load();
        }

        public TrackerFeature(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class TrackerFeatureSet extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        @Cast({"bool"})
        public native boolean addTrackerFeature(@opencv_core.Str String str);

        @Cast({"bool"})
        public native boolean addTrackerFeature(@opencv_core.Str BytePointer bytePointer);

        @Cast({"bool"})
        public native boolean addTrackerFeature(@opencv_core.Ptr @ByVal TrackerFeature trackerFeature);

        public native void extraction(@ByRef @Const opencv_core.MatVector matVector);

        @ByRef
        @Const
        public native opencv_core.MatVector getResponses();

        @ByRef
        @Const
        public native StringTrackerFeaturePairVector getTrackerFeature();

        public native void removeOutliers();

        public native void selection();

        static {
            Loader.load();
        }

        public TrackerFeatureSet(Pointer p) {
            super(p);
        }

        public TrackerFeatureSet(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public TrackerFeatureSet position(long position) {
            return (TrackerFeatureSet) super.position(position);
        }

        public TrackerFeatureSet() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv")
    public static class TrackerSamplerAlgorithm extends Pointer {
        @opencv_core.Ptr
        @ByVal
        public static native TrackerSamplerAlgorithm create(@opencv_core.Str String str);

        @opencv_core.Ptr
        @ByVal
        public static native TrackerSamplerAlgorithm create(@opencv_core.Str BytePointer bytePointer);

        @opencv_core.Str
        public native BytePointer getClassName();

        @Cast({"bool"})
        public native boolean sampling(@ByRef @Const opencv_core.Mat mat, @ByVal opencv_core.Rect rect, @ByRef opencv_core.MatVector matVector);

        static {
            Loader.load();
        }

        public TrackerSamplerAlgorithm(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class TrackerSampler extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        @Cast({"bool"})
        public native boolean addTrackerSamplerAlgorithm(@opencv_core.Str String str);

        @Cast({"bool"})
        public native boolean addTrackerSamplerAlgorithm(@opencv_core.Str BytePointer bytePointer);

        @Cast({"bool"})
        public native boolean addTrackerSamplerAlgorithm(@opencv_core.Ptr @ByVal TrackerSamplerAlgorithm trackerSamplerAlgorithm);

        @ByRef
        @Const
        public native StringTrackerSamplerAlgorithmPairVector getSamplers();

        @ByRef
        @Const
        public native opencv_core.MatVector getSamples();

        public native void sampling(@ByRef @Const opencv_core.Mat mat, @ByVal opencv_core.Rect rect);

        static {
            Loader.load();
        }

        public TrackerSampler(Pointer p) {
            super(p);
        }

        public TrackerSampler(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public TrackerSampler position(long position) {
            return (TrackerSampler) super.position(position);
        }

        public TrackerSampler() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv")
    public static class TrackerTargetState extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        public native int getTargetHeight();

        @ByVal
        public native opencv_core.Point2f getTargetPosition();

        public native int getTargetWidth();

        public native void setTargetHeight(int i);

        public native void setTargetPosition(@ByRef @Const opencv_core.Point2f point2f);

        public native void setTargetWidth(int i);

        static {
            Loader.load();
        }

        public TrackerTargetState() {
            super((Pointer) null);
            allocate();
        }

        public TrackerTargetState(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public TrackerTargetState(Pointer p) {
            super(p);
        }

        public TrackerTargetState position(long position) {
            return (TrackerTargetState) super.position(position);
        }
    }

    @Namespace("cv")
    public static class TrackerStateEstimator extends Pointer {
        @opencv_core.Ptr
        public static native TrackerStateEstimator create(@opencv_core.Str String str);

        @opencv_core.Ptr
        public static native TrackerStateEstimator create(@opencv_core.Str BytePointer bytePointer);

        @opencv_core.Ptr
        @ByVal
        public native TrackerTargetState estimate(@ByRef @Const ConfidenceMapVector confidenceMapVector);

        @opencv_core.Str
        public native BytePointer getClassName();

        public native void update(@ByRef ConfidenceMapVector confidenceMapVector);

        static {
            Loader.load();
        }

        public TrackerStateEstimator(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class TrackerModel extends Pointer {
        @ByRef
        @Const
        public native ConfidenceMapVector getConfidenceMaps();

        @ByRef
        @Const
        public native ConfidenceMap getLastConfidenceMap();

        @opencv_core.Ptr
        @ByVal
        public native TrackerTargetState getLastTargetState();

        @opencv_core.Ptr
        public native TrackerStateEstimator getTrackerStateEstimator();

        public native void modelEstimation(@ByRef @Const opencv_core.MatVector matVector);

        public native void modelUpdate();

        @Cast({"bool"})
        public native boolean runStateEstimator();

        public native void setLastTargetState(@ByRef @opencv_core.Ptr @Const TrackerTargetState trackerTargetState);

        @Cast({"bool"})
        public native boolean setTrackerStateEstimator(@opencv_core.Ptr TrackerStateEstimator trackerStateEstimator);

        static {
            Loader.load();
        }

        public TrackerModel(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class Tracker extends opencv_core.Algorithm {
        @Cast({"bool"})
        public native boolean init(@ByVal opencv_core.GpuMat gpuMat, @ByRef @Const opencv_core.Rect2d rect2d);

        @Cast({"bool"})
        public native boolean init(@ByVal opencv_core.Mat mat, @ByRef @Const opencv_core.Rect2d rect2d);

        @Cast({"bool"})
        public native boolean init(@ByVal opencv_core.UMat uMat, @ByRef @Const opencv_core.Rect2d rect2d);

        public native void read(@ByRef @Const opencv_core.FileNode fileNode);

        @Cast({"bool"})
        public native boolean update(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.Rect2d rect2d);

        @Cast({"bool"})
        public native boolean update(@ByVal opencv_core.Mat mat, @ByRef opencv_core.Rect2d rect2d);

        @Cast({"bool"})
        public native boolean update(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.Rect2d rect2d);

        public native void write(@ByRef opencv_core.FileStorage fileStorage);

        static {
            Loader.load();
        }

        public Tracker(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class TrackerStateEstimatorMILBoosting extends TrackerStateEstimator {
        private native void allocate();

        private native void allocate(int i);

        private native void allocateArray(long j);

        public native void setCurrentConfidenceMap(@ByRef ConfidenceMap confidenceMap);

        static {
            Loader.load();
        }

        public TrackerStateEstimatorMILBoosting(Pointer p) {
            super(p);
        }

        public TrackerStateEstimatorMILBoosting(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public TrackerStateEstimatorMILBoosting position(long position) {
            return (TrackerStateEstimatorMILBoosting) super.position(position);
        }

        @NoOffset
        public static class TrackerMILTargetState extends TrackerTargetState {
            private native void allocate(@ByRef @Const opencv_core.Point2f point2f, int i, int i2, @Cast({"bool"}) boolean z, @ByRef @Const opencv_core.Mat mat);

            @ByVal
            public native opencv_core.Mat getFeatures();

            @Cast({"bool"})
            public native boolean isTargetFg();

            public native void setFeatures(@ByRef @Const opencv_core.Mat mat);

            public native void setTargetFg(@Cast({"bool"}) boolean z);

            static {
                Loader.load();
            }

            public TrackerMILTargetState(Pointer p) {
                super(p);
            }

            public TrackerMILTargetState(@ByRef @Const opencv_core.Point2f position, int width, int height, @Cast({"bool"}) boolean foreground, @ByRef @Const opencv_core.Mat features) {
                super((Pointer) null);
                allocate(position, width, height, foreground, features);
            }
        }

        public TrackerStateEstimatorMILBoosting(int nFeatures) {
            super((Pointer) null);
            allocate(nFeatures);
        }

        public TrackerStateEstimatorMILBoosting() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class TrackerStateEstimatorAdaBoosting extends TrackerStateEstimator {
        private native void allocate(int i, int i2, int i3, @ByVal opencv_core.Size size, @ByRef @Const opencv_core.Rect rect);

        @StdVector
        public native IntPointer computeReplacedClassifier();

        @StdVector
        public native IntPointer computeSelectedWeakClassifier();

        @StdVector
        public native IntPointer computeSwappedClassifier();

        @ByVal
        public native opencv_core.Rect getSampleROI();

        public native void setCurrentConfidenceMap(@ByRef ConfidenceMap confidenceMap);

        public native void setSampleROI(@ByRef @Const opencv_core.Rect rect);

        static {
            Loader.load();
        }

        public TrackerStateEstimatorAdaBoosting(Pointer p) {
            super(p);
        }

        @NoOffset
        public static class TrackerAdaBoostingTargetState extends TrackerTargetState {
            private native void allocate(@ByRef @Const opencv_core.Point2f point2f, int i, int i2, @Cast({"bool"}) boolean z, @ByRef @Const opencv_core.Mat mat);

            @ByVal
            public native opencv_core.Mat getTargetResponses();

            @Cast({"bool"})
            public native boolean isTargetFg();

            public native void setTargetFg(@Cast({"bool"}) boolean z);

            public native void setTargetResponses(@ByRef @Const opencv_core.Mat mat);

            static {
                Loader.load();
            }

            public TrackerAdaBoostingTargetState(Pointer p) {
                super(p);
            }

            public TrackerAdaBoostingTargetState(@ByRef @Const opencv_core.Point2f position, int width, int height, @Cast({"bool"}) boolean foreground, @ByRef @Const opencv_core.Mat responses) {
                super((Pointer) null);
                allocate(position, width, height, foreground, responses);
            }
        }

        public TrackerStateEstimatorAdaBoosting(int numClassifer, int initIterations, int nFeatures, @ByVal opencv_core.Size patchSize, @ByRef @Const opencv_core.Rect ROI) {
            super((Pointer) null);
            allocate(numClassifer, initIterations, nFeatures, patchSize, ROI);
        }
    }

    @Namespace("cv")
    public static class TrackerStateEstimatorSVM extends TrackerStateEstimator {
        private native void allocate();

        private native void allocateArray(long j);

        static {
            Loader.load();
        }

        public TrackerStateEstimatorSVM(Pointer p) {
            super(p);
        }

        public TrackerStateEstimatorSVM(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public TrackerStateEstimatorSVM position(long position) {
            return (TrackerStateEstimatorSVM) super.position(position);
        }

        public TrackerStateEstimatorSVM() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class TrackerSamplerCSC extends TrackerSamplerAlgorithm {
        public static final int MODE_DETECT = 5;
        public static final int MODE_INIT_NEG = 2;
        public static final int MODE_INIT_POS = 1;
        public static final int MODE_TRACK_NEG = 4;
        public static final int MODE_TRACK_POS = 3;

        private native void allocate();

        private native void allocate(@ByRef(nullValue = "cv::TrackerSamplerCSC::Params()") @Const Params params);

        private native void allocateArray(long j);

        public native void setMode(int i);

        static {
            Loader.load();
        }

        public TrackerSamplerCSC(Pointer p) {
            super(p);
        }

        public TrackerSamplerCSC(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public TrackerSamplerCSC position(long position) {
            return (TrackerSamplerCSC) super.position(position);
        }

        @NoOffset
        public static class Params extends Pointer {
            private native void allocate();

            private native void allocateArray(long j);

            public native float initInRad();

            public native Params initInRad(float f);

            public native int initMaxNegNum();

            public native Params initMaxNegNum(int i);

            public native float searchWinSize();

            public native Params searchWinSize(float f);

            public native float trackInPosRad();

            public native Params trackInPosRad(float f);

            public native int trackMaxNegNum();

            public native Params trackMaxNegNum(int i);

            public native int trackMaxPosNum();

            public native Params trackMaxPosNum(int i);

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

        public TrackerSamplerCSC(@ByRef(nullValue = "cv::TrackerSamplerCSC::Params()") @Const Params parameters) {
            super((Pointer) null);
            allocate(parameters);
        }

        public TrackerSamplerCSC() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class TrackerSamplerCS extends TrackerSamplerAlgorithm {
        public static final int MODE_CLASSIFY = 3;
        public static final int MODE_NEGATIVE = 2;
        public static final int MODE_POSITIVE = 1;

        private native void allocate();

        private native void allocate(@ByRef(nullValue = "cv::TrackerSamplerCS::Params()") @Const Params params);

        private native void allocateArray(long j);

        @ByVal
        public native opencv_core.Rect getROI();

        @Cast({"bool"})
        public native boolean samplingImpl(@ByRef @Const opencv_core.Mat mat, @ByVal opencv_core.Rect rect, @ByRef opencv_core.MatVector matVector);

        public native void setMode(int i);

        static {
            Loader.load();
        }

        public TrackerSamplerCS(Pointer p) {
            super(p);
        }

        public TrackerSamplerCS(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public TrackerSamplerCS position(long position) {
            return (TrackerSamplerCS) super.position(position);
        }

        @NoOffset
        public static class Params extends Pointer {
            private native void allocate();

            private native void allocateArray(long j);

            public native float overlap();

            public native Params overlap(float f);

            public native float searchFactor();

            public native Params searchFactor(float f);

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

        public TrackerSamplerCS(@ByRef(nullValue = "cv::TrackerSamplerCS::Params()") @Const Params parameters) {
            super((Pointer) null);
            allocate(parameters);
        }

        public TrackerSamplerCS() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class TrackerSamplerPF extends TrackerSamplerAlgorithm {
        private native void allocate(@ByRef @Const opencv_core.Mat mat);

        private native void allocate(@ByRef @Const opencv_core.Mat mat, @ByRef(nullValue = "cv::TrackerSamplerPF::Params()") @Const Params params);

        static {
            Loader.load();
        }

        public TrackerSamplerPF(Pointer p) {
            super(p);
        }

        @NoOffset
        public static class Params extends Pointer {
            private native void allocate();

            private native void allocateArray(long j);

            public native double alpha();

            public native Params alpha(double d);

            public native int iterationNum();

            public native Params iterationNum(int i);

            public native int particlesNum();

            public native Params particlesNum(int i);

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

        public TrackerSamplerPF(@ByRef @Const opencv_core.Mat chosenRect, @ByRef(nullValue = "cv::TrackerSamplerPF::Params()") @Const Params parameters) {
            super((Pointer) null);
            allocate(chosenRect, parameters);
        }

        public TrackerSamplerPF(@ByRef @Const opencv_core.Mat chosenRect) {
            super((Pointer) null);
            allocate(chosenRect);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class TrackerFeatureFeature2d extends TrackerFeature {
        private native void allocate(@opencv_core.Str String str, @opencv_core.Str String str2);

        private native void allocate(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

        public native void selection(@ByRef opencv_core.Mat mat, int i);

        static {
            Loader.load();
        }

        public TrackerFeatureFeature2d(Pointer p) {
            super(p);
        }

        public TrackerFeatureFeature2d(@opencv_core.Str BytePointer detectorType, @opencv_core.Str BytePointer descriptorType) {
            super((Pointer) null);
            allocate(detectorType, descriptorType);
        }

        public TrackerFeatureFeature2d(@opencv_core.Str String detectorType, @opencv_core.Str String descriptorType) {
            super((Pointer) null);
            allocate(detectorType, descriptorType);
        }
    }

    @Namespace("cv")
    public static class TrackerFeatureHOG extends TrackerFeature {
        private native void allocate();

        private native void allocateArray(long j);

        public native void selection(@ByRef opencv_core.Mat mat, int i);

        static {
            Loader.load();
        }

        public TrackerFeatureHOG(Pointer p) {
            super(p);
        }

        public TrackerFeatureHOG(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public TrackerFeatureHOG position(long position) {
            return (TrackerFeatureHOG) super.position(position);
        }

        public TrackerFeatureHOG() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class TrackerFeatureHAAR extends TrackerFeature {
        private native void allocate();

        private native void allocate(@ByRef(nullValue = "cv::TrackerFeatureHAAR::Params()") @Const Params params);

        private native void allocateArray(long j);

        @Cast({"bool"})
        public native boolean extractSelected(@StdVector IntBuffer intBuffer, @ByRef @Const opencv_core.MatVector matVector, @ByRef opencv_core.Mat mat);

        @Cast({"bool"})
        public native boolean extractSelected(@StdVector IntPointer intPointer, @ByRef @Const opencv_core.MatVector matVector, @ByRef opencv_core.Mat mat);

        @Cast({"bool"})
        public native boolean extractSelected(@StdVector int[] iArr, @ByRef @Const opencv_core.MatVector matVector, @ByRef opencv_core.Mat mat);

        @ByRef
        public native CvHaarEvaluator.FeatureHaar getFeatureAt(int i);

        public native void selection(@ByRef opencv_core.Mat mat, int i);

        @Cast({"bool"})
        public native boolean swapFeature(int i, int i2);

        @Cast({"bool"})
        public native boolean swapFeature(int i, @ByRef CvHaarEvaluator.FeatureHaar featureHaar);

        static {
            Loader.load();
        }

        public TrackerFeatureHAAR(Pointer p) {
            super(p);
        }

        public TrackerFeatureHAAR(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public TrackerFeatureHAAR position(long position) {
            return (TrackerFeatureHAAR) super.position(position);
        }

        @NoOffset
        public static class Params extends Pointer {
            private native void allocate();

            private native void allocateArray(long j);

            public native Params isIntegral(boolean z);

            @Cast({"bool"})
            public native boolean isIntegral();

            public native int numFeatures();

            public native Params numFeatures(int i);

            @ByRef
            public native opencv_core.Size rectSize();

            public native Params rectSize(opencv_core.Size size);

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

        public TrackerFeatureHAAR(@ByRef(nullValue = "cv::TrackerFeatureHAAR::Params()") @Const Params parameters) {
            super((Pointer) null);
            allocate(parameters);
        }

        public TrackerFeatureHAAR() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv")
    public static class TrackerFeatureLBP extends TrackerFeature {
        private native void allocate();

        private native void allocateArray(long j);

        public native void selection(@ByRef opencv_core.Mat mat, int i);

        static {
            Loader.load();
        }

        public TrackerFeatureLBP(Pointer p) {
            super(p);
        }

        public TrackerFeatureLBP(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public TrackerFeatureLBP position(long position) {
            return (TrackerFeatureLBP) super.position(position);
        }

        public TrackerFeatureLBP() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv")
    public static class TrackerMIL extends Tracker {
        @opencv_core.Ptr
        public static native TrackerMIL create();

        @opencv_core.Ptr
        public static native TrackerMIL create(@ByRef @Const Params params);

        static {
            Loader.load();
        }

        public TrackerMIL(Pointer p) {
            super(p);
        }

        @NoOffset
        public static class Params extends Pointer {
            private native void allocate();

            private native void allocateArray(long j);

            public native int featureSetNumFeatures();

            public native Params featureSetNumFeatures(int i);

            public native void read(@ByRef @Const opencv_core.FileNode fileNode);

            public native float samplerInitInRadius();

            public native Params samplerInitInRadius(float f);

            public native int samplerInitMaxNegNum();

            public native Params samplerInitMaxNegNum(int i);

            public native float samplerSearchWinSize();

            public native Params samplerSearchWinSize(float f);

            public native float samplerTrackInRadius();

            public native Params samplerTrackInRadius(float f);

            public native int samplerTrackMaxNegNum();

            public native Params samplerTrackMaxNegNum(int i);

            public native int samplerTrackMaxPosNum();

            public native Params samplerTrackMaxPosNum(int i);

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
    }

    @Namespace("cv")
    public static class TrackerBoosting extends Tracker {
        @opencv_core.Ptr
        public static native TrackerBoosting create();

        @opencv_core.Ptr
        public static native TrackerBoosting create(@ByRef @Const Params params);

        static {
            Loader.load();
        }

        public TrackerBoosting(Pointer p) {
            super(p);
        }

        @NoOffset
        public static class Params extends Pointer {
            private native void allocate();

            private native void allocateArray(long j);

            public native int featureSetNumFeatures();

            public native Params featureSetNumFeatures(int i);

            public native int iterationInit();

            public native Params iterationInit(int i);

            public native int numClassifiers();

            public native Params numClassifiers(int i);

            public native void read(@ByRef @Const opencv_core.FileNode fileNode);

            public native float samplerOverlap();

            public native Params samplerOverlap(float f);

            public native float samplerSearchFactor();

            public native Params samplerSearchFactor(float f);

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
    }

    @Namespace("cv")
    public static class TrackerMedianFlow extends Tracker {
        @opencv_core.Ptr
        public static native TrackerMedianFlow create();

        @opencv_core.Ptr
        public static native TrackerMedianFlow create(@ByRef @Const Params params);

        static {
            Loader.load();
        }

        public TrackerMedianFlow(Pointer p) {
            super(p);
        }

        @NoOffset
        public static class Params extends Pointer {
            private native void allocate();

            private native void allocateArray(long j);

            public native int maxLevel();

            public native Params maxLevel(int i);

            public native double maxMedianLengthOfDisplacementDifference();

            public native Params maxMedianLengthOfDisplacementDifference(double d);

            public native int pointsInGrid();

            public native Params pointsInGrid(int i);

            public native void read(@ByRef @Const opencv_core.FileNode fileNode);

            @ByRef
            public native opencv_core.TermCriteria termCriteria();

            public native Params termCriteria(opencv_core.TermCriteria termCriteria);

            @ByRef
            public native opencv_core.Size winSize();

            public native Params winSize(opencv_core.Size size);

            @ByRef
            public native opencv_core.Size winSizeNCC();

            public native Params winSizeNCC(opencv_core.Size size);

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
    }

    @Namespace("cv")
    public static class TrackerTLD extends Tracker {
        @opencv_core.Ptr
        public static native TrackerTLD create();

        @opencv_core.Ptr
        public static native TrackerTLD create(@ByRef @Const Params params);

        static {
            Loader.load();
        }

        public TrackerTLD(Pointer p) {
            super(p);
        }

        public static class Params extends Pointer {
            private native void allocate();

            private native void allocateArray(long j);

            public native void read(@ByRef @Const opencv_core.FileNode fileNode);

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
    }

    @Namespace("cv")
    public static class TrackerKCF extends Tracker {
        public static final int CN = 2;
        public static final int CUSTOM = 4;
        public static final int GRAY = 1;

        @opencv_core.Ptr
        public static native TrackerKCF create();

        @opencv_core.Ptr
        public static native TrackerKCF create(@ByRef @Const Params params);

        public native void setFeatureExtractor(Arg0_Mat_Rect_Mat arg0_Mat_Rect_Mat);

        public native void setFeatureExtractor(Arg0_Mat_Rect_Mat arg0_Mat_Rect_Mat, @Cast({"bool"}) boolean z);

        static {
            Loader.load();
        }

        public TrackerKCF(Pointer p) {
            super(p);
        }

        @NoOffset
        public static class Params extends Pointer {
            private native void allocate();

            private native void allocateArray(long j);

            public native Params compress_feature(boolean z);

            @Cast({"bool"})
            public native boolean compress_feature();

            public native int compressed_size();

            public native Params compressed_size(int i);

            public native int desc_npca();

            public native Params desc_npca(int i);

            public native int desc_pca();

            public native Params desc_pca(int i);

            public native float detect_thresh();

            public native Params detect_thresh(float f);

            public native float interp_factor();

            public native Params interp_factor(float f);

            public native float lambda();

            public native Params lambda(float f);

            public native int max_patch_size();

            public native Params max_patch_size(int i);

            public native float output_sigma_factor();

            public native Params output_sigma_factor(float f);

            public native float pca_learning_rate();

            public native Params pca_learning_rate(float f);

            public native void read(@ByRef @Const opencv_core.FileNode fileNode);

            public native Params resize(boolean z);

            @Cast({"bool"})
            public native boolean resize();

            public native float sigma();

            public native Params sigma(float f);

            public native Params split_coeff(boolean z);

            @Cast({"bool"})
            public native boolean split_coeff();

            public native Params wrap_kernel(boolean z);

            @Cast({"bool"})
            public native boolean wrap_kernel();

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

        public static class Arg0_Mat_Rect_Mat extends FunctionPointer {
            private native void allocate();

            public native void call(@Const @ByVal opencv_core.Mat mat, @Const @ByVal opencv_core.Rect rect, @ByRef opencv_core.Mat mat2);

            static {
                Loader.load();
            }

            public Arg0_Mat_Rect_Mat(Pointer p) {
                super(p);
            }

            protected Arg0_Mat_Rect_Mat() {
                allocate();
            }
        }
    }

    @Namespace("cv")
    public static class TrackerGOTURN extends Tracker {
        @opencv_core.Ptr
        public static native TrackerGOTURN create();

        @opencv_core.Ptr
        public static native TrackerGOTURN create(@ByRef @Const Params params);

        static {
            Loader.load();
        }

        public TrackerGOTURN(Pointer p) {
            super(p);
        }

        public static class Params extends Pointer {
            private native void allocate();

            private native void allocateArray(long j);

            public native void read(@ByRef @Const opencv_core.FileNode fileNode);

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
    }

    @Namespace("cv")
    public static class TrackerMOSSE extends Tracker {
        @opencv_core.Ptr
        public static native TrackerMOSSE create();

        static {
            Loader.load();
        }

        public TrackerMOSSE(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class MultiTracker extends opencv_core.Algorithm {
        private native void allocate();

        private native void allocateArray(long j);

        @opencv_core.Ptr
        public static native MultiTracker create();

        @Cast({"bool"})
        public native boolean add(@opencv_core.Ptr @ByVal Tracker tracker, @ByVal opencv_core.GpuMat gpuMat, @ByRef @Const opencv_core.Rect2d rect2d);

        @Cast({"bool"})
        public native boolean add(@opencv_core.Ptr @ByVal Tracker tracker, @ByVal opencv_core.Mat mat, @ByRef @Const opencv_core.Rect2d rect2d);

        @Cast({"bool"})
        public native boolean add(@opencv_core.Ptr @ByVal Tracker tracker, @ByVal opencv_core.UMat uMat, @ByRef @Const opencv_core.Rect2d rect2d);

        @Cast({"bool"})
        public native boolean add(@ByVal TrackerVector trackerVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.Rect2dVector rect2dVector);

        @Cast({"bool"})
        public native boolean add(@ByVal TrackerVector trackerVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Rect2dVector rect2dVector);

        @Cast({"bool"})
        public native boolean add(@ByVal TrackerVector trackerVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.Rect2dVector rect2dVector);

        @ByRef
        @Const
        public native opencv_core.Rect2dVector getObjects();

        @Cast({"bool"})
        public native boolean update(@ByVal opencv_core.GpuMat gpuMat);

        @Cast({"bool"})
        public native boolean update(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.Rect2dVector rect2dVector);

        @Cast({"bool"})
        public native boolean update(@ByVal opencv_core.Mat mat);

        @Cast({"bool"})
        public native boolean update(@ByVal opencv_core.Mat mat, @ByRef opencv_core.Rect2dVector rect2dVector);

        @Cast({"bool"})
        public native boolean update(@ByVal opencv_core.UMat uMat);

        @Cast({"bool"})
        public native boolean update(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.Rect2dVector rect2dVector);

        static {
            Loader.load();
        }

        public MultiTracker(Pointer p) {
            super(p);
        }

        public MultiTracker(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public MultiTracker position(long position) {
            return (MultiTracker) super.position(position);
        }

        public MultiTracker() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class MultiTracker_Alt extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        @Cast({"bool"})
        public native boolean addTarget(@ByVal opencv_core.GpuMat gpuMat, @ByRef @Const opencv_core.Rect2d rect2d, @opencv_core.Ptr @ByVal Tracker tracker);

        @Cast({"bool"})
        public native boolean addTarget(@ByVal opencv_core.Mat mat, @ByRef @Const opencv_core.Rect2d rect2d, @opencv_core.Ptr @ByVal Tracker tracker);

        @Cast({"bool"})
        public native boolean addTarget(@ByVal opencv_core.UMat uMat, @ByRef @Const opencv_core.Rect2d rect2d, @opencv_core.Ptr @ByVal Tracker tracker);

        @ByRef
        public native opencv_core.Rect2dVector boundingBoxes();

        public native MultiTracker_Alt boundingBoxes(opencv_core.Rect2dVector rect2dVector);

        @ByRef
        public native opencv_core.ScalarVector colors();

        public native MultiTracker_Alt colors(opencv_core.ScalarVector scalarVector);

        public native int targetNum();

        public native MultiTracker_Alt targetNum(int i);

        public native MultiTracker_Alt trackers(TrackerVector trackerVector);

        @ByRef
        public native TrackerVector trackers();

        @Cast({"bool"})
        public native boolean update(@ByVal opencv_core.GpuMat gpuMat);

        @Cast({"bool"})
        public native boolean update(@ByVal opencv_core.Mat mat);

        @Cast({"bool"})
        public native boolean update(@ByVal opencv_core.UMat uMat);

        static {
            Loader.load();
        }

        public MultiTracker_Alt(Pointer p) {
            super(p);
        }

        public MultiTracker_Alt(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public MultiTracker_Alt position(long position) {
            return (MultiTracker_Alt) super.position(position);
        }

        public MultiTracker_Alt() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv")
    public static class MultiTrackerTLD extends MultiTracker_Alt {
        private native void allocate();

        private native void allocateArray(long j);

        @Cast({"bool"})
        public native boolean update_opt(@ByVal opencv_core.GpuMat gpuMat);

        @Cast({"bool"})
        public native boolean update_opt(@ByVal opencv_core.Mat mat);

        @Cast({"bool"})
        public native boolean update_opt(@ByVal opencv_core.UMat uMat);

        static {
            Loader.load();
        }

        public MultiTrackerTLD() {
            super((Pointer) null);
            allocate();
        }

        public MultiTrackerTLD(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public MultiTrackerTLD(Pointer p) {
            super(p);
        }

        public MultiTrackerTLD position(long position) {
            return (MultiTrackerTLD) super.position(position);
        }
    }

    @Namespace("cv")
    public static class TrackerCSRT extends Tracker {
        @opencv_core.Ptr
        public static native TrackerCSRT create();

        @opencv_core.Ptr
        public static native TrackerCSRT create(@ByRef @Const Params params);

        public native void setInitialMask(@ByVal opencv_core.GpuMat gpuMat);

        public native void setInitialMask(@ByVal opencv_core.Mat mat);

        public native void setInitialMask(@ByVal opencv_core.UMat uMat);

        static {
            Loader.load();
        }

        public TrackerCSRT(Pointer p) {
            super(p);
        }

        @NoOffset
        public static class Params extends Pointer {
            private native void allocate();

            private native void allocateArray(long j);

            public native int admm_iterations();

            public native Params admm_iterations(int i);

            public native int background_ratio();

            public native Params background_ratio(int i);

            public native float cheb_attenuation();

            public native Params cheb_attenuation(float f);

            public native float filter_lr();

            public native Params filter_lr(float f);

            public native float gsl_sigma();

            public native Params gsl_sigma(float f);

            public native int histogram_bins();

            public native Params histogram_bins(int i);

            public native float histogram_lr();

            public native Params histogram_lr(float f);

            public native float hog_clip();

            public native Params hog_clip(float f);

            public native float hog_orientations();

            public native Params hog_orientations(float f);

            public native float kaiser_alpha();

            public native Params kaiser_alpha(float f);

            public native int num_hog_channels_used();

            public native Params num_hog_channels_used(int i);

            public native int number_of_scales();

            public native Params number_of_scales(int i);

            public native float padding();

            public native Params padding(float f);

            public native float psr_threshold();

            public native Params psr_threshold(float f);

            public native void read(@ByRef @Const opencv_core.FileNode fileNode);

            public native float scale_lr();

            public native Params scale_lr(float f);

            public native float scale_model_max_area();

            public native Params scale_model_max_area(float f);

            public native float scale_sigma_factor();

            public native Params scale_sigma_factor(float f);

            public native float scale_step();

            public native Params scale_step(float f);

            public native float template_size();

            public native Params template_size(float f);

            public native Params use_channel_weights(boolean z);

            @Cast({"bool"})
            public native boolean use_channel_weights();

            public native Params use_color_names(boolean z);

            @Cast({"bool"})
            public native boolean use_color_names();

            public native Params use_gray(boolean z);

            @Cast({"bool"})
            public native boolean use_gray();

            public native Params use_hog(boolean z);

            @Cast({"bool"})
            public native boolean use_hog();

            public native Params use_rgb(boolean z);

            @Cast({"bool"})
            public native boolean use_rgb();

            public native Params use_segmentation(boolean z);

            @Cast({"bool"})
            public native boolean use_segmentation();

            public native float weights_lr();

            public native Params weights_lr(float f);

            @StdString
            public native BytePointer window_function();

            public native Params window_function(BytePointer bytePointer);

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
    }
}
