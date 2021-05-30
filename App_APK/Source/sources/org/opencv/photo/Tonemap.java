package org.opencv.photo;

import org.opencv.core.Algorithm;
import org.opencv.core.Mat;

public class Tonemap extends Algorithm {
    private static native void delete(long j);

    private static native float getGamma_0(long j);

    private static native void process_0(long j, long j2, long j3);

    private static native void setGamma_0(long j, float f);

    protected Tonemap(long addr) {
        super(addr);
    }

    public static Tonemap __fromPtr__(long addr) {
        return new Tonemap(addr);
    }

    public float getGamma() {
        return getGamma_0(this.nativeObj);
    }

    public void process(Mat src, Mat dst) {
        process_0(this.nativeObj, src.nativeObj, dst.nativeObj);
    }

    public void setGamma(float gamma) {
        setGamma_0(this.nativeObj, gamma);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
