package org.opencv.photo;

import java.util.List;
import org.opencv.core.Algorithm;
import org.opencv.core.Mat;
import org.opencv.utils.Converters;

public class AlignExposures extends Algorithm {
    private static native void delete(long j);

    private static native void process_0(long j, long j2, long j3, long j4, long j5);

    protected AlignExposures(long addr) {
        super(addr);
    }

    public static AlignExposures __fromPtr__(long addr) {
        return new AlignExposures(addr);
    }

    public void process(List<Mat> src, List<Mat> dst, Mat times, Mat response) {
        process_0(this.nativeObj, Converters.vector_Mat_to_Mat(src).nativeObj, Converters.vector_Mat_to_Mat(dst).nativeObj, times.nativeObj, response.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
