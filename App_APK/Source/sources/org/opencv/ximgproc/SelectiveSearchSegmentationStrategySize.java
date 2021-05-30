package org.opencv.ximgproc;

public class SelectiveSearchSegmentationStrategySize extends SelectiveSearchSegmentationStrategy {
    private static native void delete(long j);

    protected SelectiveSearchSegmentationStrategySize(long addr) {
        super(addr);
    }

    public static SelectiveSearchSegmentationStrategySize __fromPtr__(long addr) {
        return new SelectiveSearchSegmentationStrategySize(addr);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
