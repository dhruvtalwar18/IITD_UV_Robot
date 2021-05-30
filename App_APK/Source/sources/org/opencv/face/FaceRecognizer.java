package org.opencv.face;

import java.util.List;
import org.opencv.core.Algorithm;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.utils.Converters;

public class FaceRecognizer extends Algorithm {
    private static native void delete(long j);

    private static native String getLabelInfo_0(long j, int i);

    private static native long getLabelsByString_0(long j, String str);

    private static native void predict_0(long j, long j2, double[] dArr, double[] dArr2);

    private static native void predict_collect_0(long j, long j2, long j3);

    private static native int predict_label_0(long j, long j2);

    private static native void read_0(long j, String str);

    private static native void setLabelInfo_0(long j, int i, String str);

    private static native void train_0(long j, long j2, long j3);

    private static native void update_0(long j, long j2, long j3);

    private static native void write_0(long j, String str);

    protected FaceRecognizer(long addr) {
        super(addr);
    }

    public static FaceRecognizer __fromPtr__(long addr) {
        return new FaceRecognizer(addr);
    }

    public String getLabelInfo(int label) {
        return getLabelInfo_0(this.nativeObj, label);
    }

    public int predict_label(Mat src) {
        return predict_label_0(this.nativeObj, src.nativeObj);
    }

    public MatOfInt getLabelsByString(String str) {
        return MatOfInt.fromNativeAddr(getLabelsByString_0(this.nativeObj, str));
    }

    public void predict_collect(Mat src, PredictCollector collector) {
        predict_collect_0(this.nativeObj, src.nativeObj, collector.getNativeObjAddr());
    }

    public void predict(Mat src, int[] label, double[] confidence) {
        double[] label_out = new double[1];
        double[] confidence_out = new double[1];
        predict_0(this.nativeObj, src.nativeObj, label_out, confidence_out);
        if (label != null) {
            label[0] = (int) label_out[0];
        }
        if (confidence != null) {
            confidence[0] = confidence_out[0];
        }
    }

    public void read(String filename) {
        read_0(this.nativeObj, filename);
    }

    public void setLabelInfo(int label, String strInfo) {
        setLabelInfo_0(this.nativeObj, label, strInfo);
    }

    public void train(List<Mat> src, Mat labels) {
        train_0(this.nativeObj, Converters.vector_Mat_to_Mat(src).nativeObj, labels.nativeObj);
    }

    public void update(List<Mat> src, Mat labels) {
        update_0(this.nativeObj, Converters.vector_Mat_to_Mat(src).nativeObj, labels.nativeObj);
    }

    public void write(String filename) {
        write_0(this.nativeObj, filename);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
