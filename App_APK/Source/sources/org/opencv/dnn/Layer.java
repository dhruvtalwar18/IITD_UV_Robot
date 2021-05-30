package org.opencv.dnn;

import java.util.ArrayList;
import java.util.List;
import org.opencv.core.Algorithm;
import org.opencv.core.Mat;
import org.opencv.utils.Converters;

public class Layer extends Algorithm {
    private static native void delete(long j);

    private static native void finalize_0(long j, long j2, long j3);

    private static native long get_blobs_0(long j);

    private static native String get_name_0(long j);

    private static native int get_preferableTarget_0(long j);

    private static native String get_type_0(long j);

    private static native int outputNameToIndex_0(long j, String str);

    private static native void run_0(long j, long j2, long j3, long j4);

    private static native void set_blobs_0(long j, long j2);

    protected Layer(long addr) {
        super(addr);
    }

    public static Layer __fromPtr__(long addr) {
        return new Layer(addr);
    }

    public int outputNameToIndex(String outputName) {
        return outputNameToIndex_0(this.nativeObj, outputName);
    }

    public void finalize(List<Mat> inputs, List<Mat> outputs) {
        Mat inputs_mat = Converters.vector_Mat_to_Mat(inputs);
        Mat outputs_mat = new Mat();
        finalize_0(this.nativeObj, inputs_mat.nativeObj, outputs_mat.nativeObj);
        Converters.Mat_to_vector_Mat(outputs_mat, outputs);
        outputs_mat.release();
    }

    @Deprecated
    public void run(List<Mat> inputs, List<Mat> outputs, List<Mat> internals) {
        Mat inputs_mat = Converters.vector_Mat_to_Mat(inputs);
        Mat outputs_mat = new Mat();
        Mat internals_mat = Converters.vector_Mat_to_Mat(internals);
        run_0(this.nativeObj, inputs_mat.nativeObj, outputs_mat.nativeObj, internals_mat.nativeObj);
        Converters.Mat_to_vector_Mat(outputs_mat, outputs);
        outputs_mat.release();
        Converters.Mat_to_vector_Mat(internals_mat, internals);
        internals_mat.release();
    }

    public List<Mat> get_blobs() {
        List<Mat> retVal = new ArrayList<>();
        Converters.Mat_to_vector_Mat(new Mat(get_blobs_0(this.nativeObj)), retVal);
        return retVal;
    }

    public void set_blobs(List<Mat> blobs) {
        set_blobs_0(this.nativeObj, Converters.vector_Mat_to_Mat(blobs).nativeObj);
    }

    public String get_name() {
        return get_name_0(this.nativeObj);
    }

    public String get_type() {
        return get_type_0(this.nativeObj);
    }

    public int get_preferableTarget() {
        return get_preferableTarget_0(this.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
