package org.opencv.ximgproc;

import org.opencv.core.Algorithm;
import org.opencv.core.Mat;

public class GraphSegmentation extends Algorithm {
    private static native void delete(long j);

    private static native float getK_0(long j);

    private static native int getMinSize_0(long j);

    private static native double getSigma_0(long j);

    private static native void processImage_0(long j, long j2, long j3);

    private static native void setK_0(long j, float f);

    private static native void setMinSize_0(long j, int i);

    private static native void setSigma_0(long j, double d);

    protected GraphSegmentation(long addr) {
        super(addr);
    }

    public static GraphSegmentation __fromPtr__(long addr) {
        return new GraphSegmentation(addr);
    }

    public double getSigma() {
        return getSigma_0(this.nativeObj);
    }

    public float getK() {
        return getK_0(this.nativeObj);
    }

    public int getMinSize() {
        return getMinSize_0(this.nativeObj);
    }

    public void processImage(Mat src, Mat dst) {
        processImage_0(this.nativeObj, src.nativeObj, dst.nativeObj);
    }

    public void setK(float k) {
        setK_0(this.nativeObj, k);
    }

    public void setMinSize(int min_size) {
        setMinSize_0(this.nativeObj, min_size);
    }

    public void setSigma(double sigma) {
        setSigma_0(this.nativeObj, sigma);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
