package org.opencv.text;

public class ClassifierCallback {
    protected final long nativeObj;

    private static native void delete(long j);

    protected ClassifierCallback(long addr) {
        this.nativeObj = addr;
    }

    public long getNativeObjAddr() {
        return this.nativeObj;
    }

    public static ClassifierCallback __fromPtr__(long addr) {
        return new ClassifierCallback(addr);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
