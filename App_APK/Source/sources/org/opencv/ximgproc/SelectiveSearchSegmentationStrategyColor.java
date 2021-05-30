package org.opencv.ximgproc;

public class SelectiveSearchSegmentationStrategyColor extends SelectiveSearchSegmentationStrategy {
    private static native void delete(long j);

    protected SelectiveSearchSegmentationStrategyColor(long addr) {
        super(addr);
    }

    public static SelectiveSearchSegmentationStrategyColor __fromPtr__(long addr) {
        return new SelectiveSearchSegmentationStrategyColor(addr);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
