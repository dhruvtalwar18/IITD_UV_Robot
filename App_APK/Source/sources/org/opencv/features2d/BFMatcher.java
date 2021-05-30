package org.opencv.features2d;

public class BFMatcher extends DescriptorMatcher {
    private static native long BFMatcher_0(int i, boolean z);

    private static native long BFMatcher_1(int i);

    private static native long BFMatcher_2();

    private static native long create_0(int i, boolean z);

    private static native long create_1(int i);

    private static native long create_2();

    private static native void delete(long j);

    protected BFMatcher(long addr) {
        super(addr);
    }

    public static BFMatcher __fromPtr__(long addr) {
        return new BFMatcher(addr);
    }

    public BFMatcher(int normType, boolean crossCheck) {
        super(BFMatcher_0(normType, crossCheck));
    }

    public BFMatcher(int normType) {
        super(BFMatcher_1(normType));
    }

    public BFMatcher() {
        super(BFMatcher_2());
    }

    public static BFMatcher create(int normType, boolean crossCheck) {
        return __fromPtr__(create_0(normType, crossCheck));
    }

    public static BFMatcher create(int normType) {
        return __fromPtr__(create_1(normType));
    }

    public static BFMatcher create() {
        return __fromPtr__(create_2());
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
