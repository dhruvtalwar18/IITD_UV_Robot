package org.opencv.face;

public class FacemarkTrain extends Facemark {
    private static native void delete(long j);

    protected FacemarkTrain(long addr) {
        super(addr);
    }

    public static FacemarkTrain __fromPtr__(long addr) {
        return new FacemarkTrain(addr);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
