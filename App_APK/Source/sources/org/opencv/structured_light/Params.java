package org.opencv.structured_light;

public class Params {
    protected final long nativeObj;

    private static native long Params_0();

    private static native void delete(long j);

    protected Params(long addr) {
        this.nativeObj = addr;
    }

    public long getNativeObjAddr() {
        return this.nativeObj;
    }

    public static Params __fromPtr__(long addr) {
        return new Params(addr);
    }

    public Params() {
        this.nativeObj = Params_0();
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
