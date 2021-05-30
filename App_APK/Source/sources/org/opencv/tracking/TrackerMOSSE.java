package org.opencv.tracking;

public class TrackerMOSSE extends Tracker {
    private static native long create_0();

    private static native void delete(long j);

    protected TrackerMOSSE(long addr) {
        super(addr);
    }

    public static TrackerMOSSE __fromPtr__(long addr) {
        return new TrackerMOSSE(addr);
    }

    public static TrackerMOSSE create() {
        return __fromPtr__(create_0());
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
