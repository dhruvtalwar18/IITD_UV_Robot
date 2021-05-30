package org.opencv.tracking;

public class TrackerMIL extends Tracker {
    private static native long create_0();

    private static native void delete(long j);

    protected TrackerMIL(long addr) {
        super(addr);
    }

    public static TrackerMIL __fromPtr__(long addr) {
        return new TrackerMIL(addr);
    }

    public static TrackerMIL create() {
        return __fromPtr__(create_0());
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
