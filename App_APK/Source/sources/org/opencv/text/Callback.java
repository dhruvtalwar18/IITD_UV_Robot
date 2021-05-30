package org.opencv.text;

public class Callback {
    protected final long nativeObj;

    private static native void delete(long j);

    protected Callback(long addr) {
        this.nativeObj = addr;
    }

    public long getNativeObjAddr() {
        return this.nativeObj;
    }

    public static Callback __fromPtr__(long addr) {
        return new Callback(addr);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
