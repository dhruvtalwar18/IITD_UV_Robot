package org.opencv.ximgproc;

public class SelectiveSearchSegmentationStrategyMultiple extends SelectiveSearchSegmentationStrategy {
    private static native void addStrategy_0(long j, long j2, float f);

    private static native void clearStrategies_0(long j);

    private static native void delete(long j);

    protected SelectiveSearchSegmentationStrategyMultiple(long addr) {
        super(addr);
    }

    public static SelectiveSearchSegmentationStrategyMultiple __fromPtr__(long addr) {
        return new SelectiveSearchSegmentationStrategyMultiple(addr);
    }

    public void addStrategy(SelectiveSearchSegmentationStrategy g, float weight) {
        addStrategy_0(this.nativeObj, g.getNativeObjAddr(), weight);
    }

    public void clearStrategies() {
        clearStrategies_0(this.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
