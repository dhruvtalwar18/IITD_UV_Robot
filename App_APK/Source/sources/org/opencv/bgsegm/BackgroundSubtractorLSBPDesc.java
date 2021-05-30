package org.opencv.bgsegm;

public class BackgroundSubtractorLSBPDesc {
    protected final long nativeObj;

    private static native void delete(long j);

    protected BackgroundSubtractorLSBPDesc(long addr) {
        this.nativeObj = addr;
    }

    public long getNativeObjAddr() {
        return this.nativeObj;
    }

    public static BackgroundSubtractorLSBPDesc __fromPtr__(long addr) {
        return new BackgroundSubtractorLSBPDesc(addr);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
