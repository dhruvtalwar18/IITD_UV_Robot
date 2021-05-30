package org.opencv.photo;

import java.util.List;
import org.opencv.core.Algorithm;
import org.opencv.core.Mat;
import org.opencv.utils.Converters;

public class MergeExposures extends Algorithm {
    private static native void delete(long j);

    private static native void process_0(long j, long j2, long j3, long j4, long j5);

    protected MergeExposures(long addr) {
        super(addr);
    }

    public static MergeExposures __fromPtr__(long addr) {
        return new MergeExposures(addr);
    }

    public void process(List<Mat> src, Mat dst, Mat times, Mat response) {
        process_0(this.nativeObj, Converters.vector_Mat_to_Mat(src).nativeObj, dst.nativeObj, times.nativeObj, response.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
