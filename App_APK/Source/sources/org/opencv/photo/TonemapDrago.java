package org.opencv.photo;

public class TonemapDrago extends Tonemap {
    private static native void delete(long j);

    private static native float getBias_0(long j);

    private static native float getSaturation_0(long j);

    private static native void setBias_0(long j, float f);

    private static native void setSaturation_0(long j, float f);

    protected TonemapDrago(long addr) {
        super(addr);
    }

    public static TonemapDrago __fromPtr__(long addr) {
        return new TonemapDrago(addr);
    }

    public float getBias() {
        return getBias_0(this.nativeObj);
    }

    public float getSaturation() {
        return getSaturation_0(this.nativeObj);
    }

    public void setBias(float bias) {
        setBias_0(this.nativeObj, bias);
    }

    public void setSaturation(float saturation) {
        setSaturation_0(this.nativeObj, saturation);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
