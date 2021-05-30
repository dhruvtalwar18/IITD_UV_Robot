package org.opencv.face;

public class PredictCollector {
    protected final long nativeObj;

    private static native void delete(long j);

    protected PredictCollector(long addr) {
        this.nativeObj = addr;
    }

    public long getNativeObjAddr() {
        return this.nativeObj;
    }

    public static PredictCollector __fromPtr__(long addr) {
        return new PredictCollector(addr);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
