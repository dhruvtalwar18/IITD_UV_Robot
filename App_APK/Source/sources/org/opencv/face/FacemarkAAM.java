package org.opencv.face;

public class FacemarkAAM extends FacemarkTrain {
    private static native void delete(long j);

    protected FacemarkAAM(long addr) {
        super(addr);
    }

    public static FacemarkAAM __fromPtr__(long addr) {
        return new FacemarkAAM(addr);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
