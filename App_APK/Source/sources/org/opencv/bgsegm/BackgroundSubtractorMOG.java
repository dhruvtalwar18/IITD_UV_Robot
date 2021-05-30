package org.opencv.bgsegm;

import org.opencv.video.BackgroundSubtractor;

public class BackgroundSubtractorMOG extends BackgroundSubtractor {
    private static native void delete(long j);

    private static native double getBackgroundRatio_0(long j);

    private static native int getHistory_0(long j);

    private static native int getNMixtures_0(long j);

    private static native double getNoiseSigma_0(long j);

    private static native void setBackgroundRatio_0(long j, double d);

    private static native void setHistory_0(long j, int i);

    private static native void setNMixtures_0(long j, int i);

    private static native void setNoiseSigma_0(long j, double d);

    protected BackgroundSubtractorMOG(long addr) {
        super(addr);
    }

    public static BackgroundSubtractorMOG __fromPtr__(long addr) {
        return new BackgroundSubtractorMOG(addr);
    }

    public double getBackgroundRatio() {
        return getBackgroundRatio_0(this.nativeObj);
    }

    public double getNoiseSigma() {
        return getNoiseSigma_0(this.nativeObj);
    }

    public int getHistory() {
        return getHistory_0(this.nativeObj);
    }

    public int getNMixtures() {
        return getNMixtures_0(this.nativeObj);
    }

    public void setBackgroundRatio(double backgroundRatio) {
        setBackgroundRatio_0(this.nativeObj, backgroundRatio);
    }

    public void setHistory(int nframes) {
        setHistory_0(this.nativeObj, nframes);
    }

    public void setNMixtures(int nmix) {
        setNMixtures_0(this.nativeObj, nmix);
    }

    public void setNoiseSigma(double noiseSigma) {
        setNoiseSigma_0(this.nativeObj, noiseSigma);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
