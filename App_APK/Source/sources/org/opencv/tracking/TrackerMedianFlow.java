package org.opencv.tracking;

public class TrackerMedianFlow extends Tracker {
    private static native long create_0();

    private static native void delete(long j);

    protected TrackerMedianFlow(long addr) {
        super(addr);
    }

    public static TrackerMedianFlow __fromPtr__(long addr) {
        return new TrackerMedianFlow(addr);
    }

    public static TrackerMedianFlow create() {
        return __fromPtr__(create_0());
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
