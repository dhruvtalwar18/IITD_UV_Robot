package org.opencv.tracking;

public class TrackerBoosting extends Tracker {
    private static native long create_0();

    private static native void delete(long j);

    protected TrackerBoosting(long addr) {
        super(addr);
    }

    public static TrackerBoosting __fromPtr__(long addr) {
        return new TrackerBoosting(addr);
    }

    public static TrackerBoosting create() {
        return __fromPtr__(create_0());
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
