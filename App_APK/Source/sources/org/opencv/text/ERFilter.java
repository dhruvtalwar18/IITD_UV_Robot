package org.opencv.text;

import org.opencv.core.Algorithm;

public class ERFilter extends Algorithm {
    private static native void delete(long j);

    protected ERFilter(long addr) {
        super(addr);
    }

    public static ERFilter __fromPtr__(long addr) {
        return new ERFilter(addr);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
