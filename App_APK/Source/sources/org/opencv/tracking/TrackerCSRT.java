package org.opencv.tracking;

import org.opencv.core.Mat;

public class TrackerCSRT extends Tracker {
    private static native long create_0();

    private static native void delete(long j);

    private static native void setInitialMask_0(long j, long j2);

    protected TrackerCSRT(long addr) {
        super(addr);
    }

    public static TrackerCSRT __fromPtr__(long addr) {
        return new TrackerCSRT(addr);
    }

    public static TrackerCSRT create() {
        return __fromPtr__(create_0());
    }

    public void setInitialMask(Mat mask) {
        setInitialMask_0(this.nativeObj, mask.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
