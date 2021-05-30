package org.opencv.ximgproc;

import org.opencv.core.Algorithm;
import org.opencv.core.Mat;

public class DTFilter extends Algorithm {
    private static native void delete(long j);

    private static native void filter_0(long j, long j2, long j3, int i);

    private static native void filter_1(long j, long j2, long j3);

    protected DTFilter(long addr) {
        super(addr);
    }

    public static DTFilter __fromPtr__(long addr) {
        return new DTFilter(addr);
    }

    public void filter(Mat src, Mat dst, int dDepth) {
        filter_0(this.nativeObj, src.nativeObj, dst.nativeObj, dDepth);
    }

    public void filter(Mat src, Mat dst) {
        filter_1(this.nativeObj, src.nativeObj, dst.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}