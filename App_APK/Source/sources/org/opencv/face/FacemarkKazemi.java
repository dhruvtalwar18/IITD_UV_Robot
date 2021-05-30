package org.opencv.face;

public class FacemarkKazemi extends Facemark {
    private static native void delete(long j);

    protected FacemarkKazemi(long addr) {
        super(addr);
    }

    public static FacemarkKazemi __fromPtr__(long addr) {
        return new FacemarkKazemi(addr);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
