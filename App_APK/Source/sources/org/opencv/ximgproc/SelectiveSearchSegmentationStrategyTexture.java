package org.opencv.ximgproc;

public class SelectiveSearchSegmentationStrategyTexture extends SelectiveSearchSegmentationStrategy {
    private static native void delete(long j);

    protected SelectiveSearchSegmentationStrategyTexture(long addr) {
        super(addr);
    }

    public static SelectiveSearchSegmentationStrategyTexture __fromPtr__(long addr) {
        return new SelectiveSearchSegmentationStrategyTexture(addr);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
