package org.opencv.xphoto;

import org.opencv.core.Algorithm;
import org.opencv.core.Mat;

public class WhiteBalancer extends Algorithm {
    private static native void balanceWhite_0(long j, long j2, long j3);

    private static native void delete(long j);

    protected WhiteBalancer(long addr) {
        super(addr);
    }

    public static WhiteBalancer __fromPtr__(long addr) {
        return new WhiteBalancer(addr);
    }

    public void balanceWhite(Mat src, Mat dst) {
        balanceWhite_0(this.nativeObj, src.nativeObj, dst.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
