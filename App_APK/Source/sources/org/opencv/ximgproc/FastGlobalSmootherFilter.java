package org.opencv.ximgproc;

import org.opencv.core.Algorithm;
import org.opencv.core.Mat;

public class FastGlobalSmootherFilter extends Algorithm {
    private static native void delete(long j);

    private static native void filter_0(long j, long j2, long j3);

    protected FastGlobalSmootherFilter(long addr) {
        super(addr);
    }

    public static FastGlobalSmootherFilter __fromPtr__(long addr) {
        return new FastGlobalSmootherFilter(addr);
    }

    public void filter(Mat src, Mat dst) {
        filter_0(this.nativeObj, src.nativeObj, dst.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
