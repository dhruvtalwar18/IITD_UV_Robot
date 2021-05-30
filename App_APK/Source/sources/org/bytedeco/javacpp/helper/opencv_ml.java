package org.bytedeco.javacpp.helper;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.Loader;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacpp.annotation.Name;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.opencv_ml;
import org.bytedeco.javacpp.presets.opencv_core;

public class opencv_ml extends org.bytedeco.javacpp.presets.opencv_ml {

    @Name({"cv::ml::StatModel"})
    public static abstract class AbstractStatModel extends opencv_core.Algorithm {
        @opencv_core.Ptr
        @Name({"load<cv::ml::ANN_MLP>"})
        public static native opencv_ml.ANN_MLP loadANN_MLP(@opencv_core.Str String str, @opencv_core.Str String str2);

        @opencv_core.Ptr
        @Name({"load<cv::ml::ANN_MLP>"})
        public static native opencv_ml.ANN_MLP loadANN_MLP(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

        @opencv_core.Ptr
        @Name({"load<cv::ml::Boost>"})
        public static native opencv_ml.Boost loadBoost(@opencv_core.Str String str, @opencv_core.Str String str2);

        @opencv_core.Ptr
        @Name({"load<cv::ml::Boost>"})
        public static native opencv_ml.Boost loadBoost(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

        @opencv_core.Ptr
        @Name({"load<cv::ml::DTrees>"})
        public static native opencv_ml.DTrees loadDTrees(@opencv_core.Str String str, @opencv_core.Str String str2);

        @opencv_core.Ptr
        @Name({"load<cv::ml::DTrees>"})
        public static native opencv_ml.DTrees loadDTrees(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

        @opencv_core.Ptr
        @Name({"load<cv::ml::EM>"})
        public static native opencv_ml.EM loadEM(@opencv_core.Str String str, @opencv_core.Str String str2);

        @opencv_core.Ptr
        @Name({"load<cv::ml::EM>"})
        public static native opencv_ml.EM loadEM(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

        @opencv_core.Ptr
        @Name({"load<cv::ml::KNearest>"})
        public static native opencv_ml.KNearest loadKNearest(@opencv_core.Str String str, @opencv_core.Str String str2);

        @opencv_core.Ptr
        @Name({"load<cv::ml::KNearest>"})
        public static native opencv_ml.KNearest loadKNearest(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

        @opencv_core.Ptr
        @Name({"load<cv::ml::LogisticRegression>"})
        public static native opencv_ml.LogisticRegression loadLogisticRegression(@opencv_core.Str String str, @opencv_core.Str String str2);

        @opencv_core.Ptr
        @Name({"load<cv::ml::LogisticRegression>"})
        public static native opencv_ml.LogisticRegression loadLogisticRegression(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

        @opencv_core.Ptr
        @Name({"load<cv::ml::NormalBayesClassifier>"})
        public static native opencv_ml.NormalBayesClassifier loadNormalBayesClassifier(@opencv_core.Str String str, @opencv_core.Str String str2);

        @opencv_core.Ptr
        @Name({"load<cv::ml::NormalBayesClassifier>"})
        public static native opencv_ml.NormalBayesClassifier loadNormalBayesClassifier(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

        @opencv_core.Ptr
        @Name({"load<cv::ml::RTrees>"})
        public static native opencv_ml.RTrees loadRTrees(@opencv_core.Str String str, @opencv_core.Str String str2);

        @opencv_core.Ptr
        @Name({"load<cv::ml::RTrees>"})
        public static native opencv_ml.RTrees loadRTrees(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

        @opencv_core.Ptr
        @Name({"load<cv::ml::SVM>"})
        public static native opencv_ml.SVM loadSVM(@opencv_core.Str String str, @opencv_core.Str String str2);

        @opencv_core.Ptr
        @Name({"load<cv::ml::SVM>"})
        public static native opencv_ml.SVM loadSVM(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

        static {
            Loader.load();
        }

        public AbstractStatModel(Pointer p) {
            super(p);
        }
    }
}
