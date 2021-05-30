package org.opencv.ximgproc;

import org.opencv.core.Algorithm;
import org.opencv.core.Mat;

public class FastBilateralSolverFilter extends Algorithm {
    private static native void delete(long j);

    private static native void filter_0(long j, long j2, long j3, long j4);

    protected FastBilateralSolverFilter(long addr) {
        super(addr);
    }

    public static FastBilateralSolverFilter __fromPtr__(long addr) {
        return new FastBilateralSolverFilter(addr);
    }

    public void filter(Mat src, Mat confidence, Mat dst) {
        filter_0(this.nativeObj, src.nativeObj, confidence.nativeObj, dst.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
