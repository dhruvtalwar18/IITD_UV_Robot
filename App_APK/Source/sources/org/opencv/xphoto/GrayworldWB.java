package org.opencv.xphoto;

public class GrayworldWB extends WhiteBalancer {
    private static native void delete(long j);

    private static native float getSaturationThreshold_0(long j);

    private static native void setSaturationThreshold_0(long j, float f);

    protected GrayworldWB(long addr) {
        super(addr);
    }

    public static GrayworldWB __fromPtr__(long addr) {
        return new GrayworldWB(addr);
    }

    public float getSaturationThreshold() {
        return getSaturationThreshold_0(this.nativeObj);
    }

    public void setSaturationThreshold(float val) {
        setSaturationThreshold_0(this.nativeObj, val);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
