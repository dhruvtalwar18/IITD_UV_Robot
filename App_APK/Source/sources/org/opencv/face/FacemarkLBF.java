package org.opencv.face;

public class FacemarkLBF extends FacemarkTrain {
    private static native void delete(long j);

    protected FacemarkLBF(long addr) {
        super(addr);
    }

    public static FacemarkLBF __fromPtr__(long addr) {
        return new FacemarkLBF(addr);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
