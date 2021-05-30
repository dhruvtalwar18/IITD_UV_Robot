package org.opencv.photo;

import java.util.List;
import org.opencv.core.Mat;
import org.opencv.utils.Converters;

public class MergeRobertson extends MergeExposures {
    private static native void delete(long j);

    private static native void process_0(long j, long j2, long j3, long j4, long j5);

    private static native void process_1(long j, long j2, long j3, long j4);

    protected MergeRobertson(long addr) {
        super(addr);
    }

    public static MergeRobertson __fromPtr__(long addr) {
        return new MergeRobertson(addr);
    }

    public void process(List<Mat> src, Mat dst, Mat times, Mat response) {
        process_0(this.nativeObj, Converters.vector_Mat_to_Mat(src).nativeObj, dst.nativeObj, times.nativeObj, response.nativeObj);
    }

    public void process(List<Mat> src, Mat dst, Mat times) {
        process_1(this.nativeObj, Converters.vector_Mat_to_Mat(src).nativeObj, dst.nativeObj, times.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
