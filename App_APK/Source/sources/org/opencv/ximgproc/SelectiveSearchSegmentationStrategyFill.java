package org.opencv.ximgproc;

public class SelectiveSearchSegmentationStrategyFill extends SelectiveSearchSegmentationStrategy {
    private static native void delete(long j);

    protected SelectiveSearchSegmentationStrategyFill(long addr) {
        super(addr);
    }

    public static SelectiveSearchSegmentationStrategyFill __fromPtr__(long addr) {
        return new SelectiveSearchSegmentationStrategyFill(addr);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
