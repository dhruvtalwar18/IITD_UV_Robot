package org.opencv.tracking;

public class TrackerKCF extends Tracker {
    public static final int CN = 2;
    public static final int CUSTOM = 4;
    public static final int GRAY = 1;

    private static native long create_0();

    private static native void delete(long j);

    protected TrackerKCF(long addr) {
        super(addr);
    }

    public static TrackerKCF __fromPtr__(long addr) {
        return new TrackerKCF(addr);
    }

    public static TrackerKCF create() {
        return __fromPtr__(create_0());
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
