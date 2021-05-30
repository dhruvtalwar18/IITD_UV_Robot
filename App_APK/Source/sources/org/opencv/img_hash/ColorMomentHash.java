package org.opencv.img_hash;

public class ColorMomentHash extends ImgHashBase {
    private static native long create_0();

    private static native void delete(long j);

    protected ColorMomentHash(long addr) {
        super(addr);
    }

    public static ColorMomentHash __fromPtr__(long addr) {
        return new ColorMomentHash(addr);
    }

    public static ColorMomentHash create() {
        return __fromPtr__(create_0());
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
