package org.opencv.photo;

import java.util.List;
import org.opencv.core.Algorithm;
import org.opencv.core.Mat;
import org.opencv.utils.Converters;

public class CalibrateCRF extends Algorithm {
    private static native void delete(long j);

    private static native void process_0(long j, long j2, long j3, long j4);

    protected CalibrateCRF(long addr) {
        super(addr);
    }

    public static CalibrateCRF __fromPtr__(long addr) {
        return new CalibrateCRF(addr);
    }

    public void process(List<Mat> src, Mat dst, Mat times) {
        process_0(this.nativeObj, Converters.vector_Mat_to_Mat(src).nativeObj, dst.nativeObj, times.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
