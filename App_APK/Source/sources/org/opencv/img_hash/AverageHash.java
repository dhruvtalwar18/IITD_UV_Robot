package org.opencv.img_hash;

public class AverageHash extends ImgHashBase {
    private static native long create_0();

    private static native void delete(long j);

    protected AverageHash(long addr) {
        super(addr);
    }

    public static AverageHash __fromPtr__(long addr) {
        return new AverageHash(addr);
    }

    public static AverageHash create() {
        return __fromPtr__(create_0());
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
