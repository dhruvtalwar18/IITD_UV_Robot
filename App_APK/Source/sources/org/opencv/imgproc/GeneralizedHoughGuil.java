package org.opencv.imgproc;

public class GeneralizedHoughGuil extends GeneralizedHough {
    private static native void delete(long j);

    protected GeneralizedHoughGuil(long addr) {
        super(addr);
    }

    public static GeneralizedHoughGuil __fromPtr__(long addr) {
        return new GeneralizedHoughGuil(addr);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
