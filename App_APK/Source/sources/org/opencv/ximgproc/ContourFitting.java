package org.opencv.ximgproc;

import org.opencv.core.Algorithm;
import org.opencv.core.Mat;

public class ContourFitting extends Algorithm {
    private static native void delete(long j);

    private static native void estimateTransformation_0(long j, long j2, long j3, long j4, double[] dArr, boolean z);

    private static native void estimateTransformation_1(long j, long j2, long j3, long j4, double[] dArr);

    private static native int getCtrSize_0(long j);

    private static native int getFDSize_0(long j);

    private static native void setCtrSize_0(long j, int i);

    private static native void setFDSize_0(long j, int i);

    protected ContourFitting(long addr) {
        super(addr);
    }

    public static ContourFitting __fromPtr__(long addr) {
        return new ContourFitting(addr);
    }

    public int getCtrSize() {
        return getCtrSize_0(this.nativeObj);
    }

    public int getFDSize() {
        return getFDSize_0(this.nativeObj);
    }

    public void estimateTransformation(Mat src, Mat dst, Mat alphaPhiST, double[] dist, boolean fdContour) {
        double[] dist_out = new double[1];
        estimateTransformation_0(this.nativeObj, src.nativeObj, dst.nativeObj, alphaPhiST.nativeObj, dist_out, fdContour);
        if (dist != null) {
            dist[0] = dist_out[0];
        }
    }

    public void estimateTransformation(Mat src, Mat dst, Mat alphaPhiST, double[] dist) {
        double[] dist_out = new double[1];
        estimateTransformation_1(this.nativeObj, src.nativeObj, dst.nativeObj, alphaPhiST.nativeObj, dist_out);
        if (dist != null) {
            dist[0] = dist_out[0];
        }
    }

    public void setCtrSize(int n) {
        setCtrSize_0(this.nativeObj, n);
    }

    public void setFDSize(int n) {
        setFDSize_0(this.nativeObj, n);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
