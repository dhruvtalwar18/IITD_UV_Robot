package org.opencv.tracking;

public class TrackerTLD extends Tracker {
    private static native long create_0();

    private static native void delete(long j);

    protected TrackerTLD(long addr) {
        super(addr);
    }

    public static TrackerTLD __fromPtr__(long addr) {
        return new TrackerTLD(addr);
    }

    public static TrackerTLD create() {
        return __fromPtr__(create_0());
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
