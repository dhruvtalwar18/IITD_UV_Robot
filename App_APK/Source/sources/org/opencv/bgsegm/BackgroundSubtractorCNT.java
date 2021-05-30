package org.opencv.bgsegm;

import org.opencv.core.Mat;
import org.opencv.video.BackgroundSubtractor;

public class BackgroundSubtractorCNT extends BackgroundSubtractor {
    private static native void apply_0(long j, long j2, long j3, double d);

    private static native void apply_1(long j, long j2, long j3);

    private static native void delete(long j);

    private static native void getBackgroundImage_0(long j, long j2);

    private static native boolean getIsParallel_0(long j);

    private static native int getMaxPixelStability_0(long j);

    private static native int getMinPixelStability_0(long j);

    private static native boolean getUseHistory_0(long j);

    private static native void setIsParallel_0(long j, boolean z);

    private static native void setMaxPixelStability_0(long j, int i);

    private static native void setMinPixelStability_0(long j, int i);

    private static native void setUseHistory_0(long j, boolean z);

    protected BackgroundSubtractorCNT(long addr) {
        super(addr);
    }

    public static BackgroundSubtractorCNT __fromPtr__(long addr) {
        return new BackgroundSubtractorCNT(addr);
    }

    public boolean getIsParallel() {
        return getIsParallel_0(this.nativeObj);
    }

    public boolean getUseHistory() {
        return getUseHistory_0(this.nativeObj);
    }

    public int getMaxPixelStability() {
        return getMaxPixelStability_0(this.nativeObj);
    }

    public int getMinPixelStability() {
        return getMinPixelStability_0(this.nativeObj);
    }

    public void apply(Mat image, Mat fgmask, double learningRate) {
        apply_0(this.nativeObj, image.nativeObj, fgmask.nativeObj, learningRate);
    }

    public void apply(Mat image, Mat fgmask) {
        apply_1(this.nativeObj, image.nativeObj, fgmask.nativeObj);
    }

    public void getBackgroundImage(Mat backgroundImage) {
        getBackgroundImage_0(this.nativeObj, backgroundImage.nativeObj);
    }

    public void setIsParallel(boolean value) {
        setIsParallel_0(this.nativeObj, value);
    }

    public void setMaxPixelStability(int value) {
        setMaxPixelStability_0(this.nativeObj, value);
    }

    public void setMinPixelStability(int value) {
        setMinPixelStability_0(this.nativeObj, value);
    }

    public void setUseHistory(boolean value) {
        setUseHistory_0(this.nativeObj, value);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
