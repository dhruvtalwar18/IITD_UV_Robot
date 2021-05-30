package org.opencv.ximgproc;

import org.opencv.core.Algorithm;
import org.opencv.core.Mat;

public class RFFeatureGetter extends Algorithm {
    private static native void delete(long j);

    private static native void getFeatures_0(long j, long j2, long j3, int i, int i2, int i3, int i4, int i5);

    protected RFFeatureGetter(long addr) {
        super(addr);
    }

    public static RFFeatureGetter __fromPtr__(long addr) {
        return new RFFeatureGetter(addr);
    }

    public void getFeatures(Mat src, Mat features, int gnrmRad, int gsmthRad, int shrink, int outNum, int gradNum) {
        getFeatures_0(this.nativeObj, src.nativeObj, features.nativeObj, gnrmRad, gsmthRad, shrink, outNum, gradNum);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
