package org.opencv.xphoto;

import org.opencv.core.Mat;

public class LearningBasedWB extends WhiteBalancer {
    private static native void delete(long j);

    private static native void extractSimpleFeatures_0(long j, long j2, long j3);

    private static native int getHistBinNum_0(long j);

    private static native int getRangeMaxVal_0(long j);

    private static native float getSaturationThreshold_0(long j);

    private static native void setHistBinNum_0(long j, int i);

    private static native void setRangeMaxVal_0(long j, int i);

    private static native void setSaturationThreshold_0(long j, float f);

    protected LearningBasedWB(long addr) {
        super(addr);
    }

    public static LearningBasedWB __fromPtr__(long addr) {
        return new LearningBasedWB(addr);
    }

    public float getSaturationThreshold() {
        return getSaturationThreshold_0(this.nativeObj);
    }

    public int getHistBinNum() {
        return getHistBinNum_0(this.nativeObj);
    }

    public int getRangeMaxVal() {
        return getRangeMaxVal_0(this.nativeObj);
    }

    public void extractSimpleFeatures(Mat src, Mat dst) {
        extractSimpleFeatures_0(this.nativeObj, src.nativeObj, dst.nativeObj);
    }

    public void setHistBinNum(int val) {
        setHistBinNum_0(this.nativeObj, val);
    }

    public void setRangeMaxVal(int val) {
        setRangeMaxVal_0(this.nativeObj, val);
    }

    public void setSaturationThreshold(float val) {
        setSaturationThreshold_0(this.nativeObj, val);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
