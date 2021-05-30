package org.opencv.ximgproc;

import org.opencv.core.Algorithm;
import org.opencv.core.Mat;

public class AdaptiveManifoldFilter extends Algorithm {
    private static native void collectGarbage_0(long j);

    private static native long create_0();

    private static native void delete(long j);

    private static native void filter_0(long j, long j2, long j3, long j4);

    private static native void filter_1(long j, long j2, long j3);

    protected AdaptiveManifoldFilter(long addr) {
        super(addr);
    }

    public static AdaptiveManifoldFilter __fromPtr__(long addr) {
        return new AdaptiveManifoldFilter(addr);
    }

    public static AdaptiveManifoldFilter create() {
        return __fromPtr__(create_0());
    }

    public void collectGarbage() {
        collectGarbage_0(this.nativeObj);
    }

    public void filter(Mat src, Mat dst, Mat joint) {
        filter_0(this.nativeObj, src.nativeObj, dst.nativeObj, joint.nativeObj);
    }

    public void filter(Mat src, Mat dst) {
        filter_1(this.nativeObj, src.nativeObj, dst.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
