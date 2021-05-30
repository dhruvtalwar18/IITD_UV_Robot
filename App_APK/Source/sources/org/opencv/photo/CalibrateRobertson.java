package org.opencv.photo;

import org.opencv.core.Mat;

public class CalibrateRobertson extends CalibrateCRF {
    private static native void delete(long j);

    private static native int getMaxIter_0(long j);

    private static native long getRadiance_0(long j);

    private static native float getThreshold_0(long j);

    private static native void setMaxIter_0(long j, int i);

    private static native void setThreshold_0(long j, float f);

    protected CalibrateRobertson(long addr) {
        super(addr);
    }

    public static CalibrateRobertson __fromPtr__(long addr) {
        return new CalibrateRobertson(addr);
    }

    public Mat getRadiance() {
        return new Mat(getRadiance_0(this.nativeObj));
    }

    public float getThreshold() {
        return getThreshold_0(this.nativeObj);
    }

    public int getMaxIter() {
        return getMaxIter_0(this.nativeObj);
    }

    public void setMaxIter(int max_iter) {
        setMaxIter_0(this.nativeObj, max_iter);
    }

    public void setThreshold(float threshold) {
        setThreshold_0(this.nativeObj, threshold);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
