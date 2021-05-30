package org.opencv.face;

public class EigenFaceRecognizer extends BasicFaceRecognizer {
    private static native long create_0(int i, double d);

    private static native long create_1(int i);

    private static native long create_2();

    private static native void delete(long j);

    protected EigenFaceRecognizer(long addr) {
        super(addr);
    }

    public static EigenFaceRecognizer __fromPtr__(long addr) {
        return new EigenFaceRecognizer(addr);
    }

    public static EigenFaceRecognizer create(int num_components, double threshold) {
        return __fromPtr__(create_0(num_components, threshold));
    }

    public static EigenFaceRecognizer create(int num_components) {
        return __fromPtr__(create_1(num_components));
    }

    public static EigenFaceRecognizer create() {
        return __fromPtr__(create_2());
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
