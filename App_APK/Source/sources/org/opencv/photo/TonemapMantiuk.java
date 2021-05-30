package org.opencv.photo;

public class TonemapMantiuk extends Tonemap {
    private static native void delete(long j);

    private static native float getSaturation_0(long j);

    private static native float getScale_0(long j);

    private static native void setSaturation_0(long j, float f);

    private static native void setScale_0(long j, float f);

    protected TonemapMantiuk(long addr) {
        super(addr);
    }

    public static TonemapMantiuk __fromPtr__(long addr) {
        return new TonemapMantiuk(addr);
    }

    public float getSaturation() {
        return getSaturation_0(this.nativeObj);
    }

    public float getScale() {
        return getScale_0(this.nativeObj);
    }

    public void setSaturation(float saturation) {
        setSaturation_0(this.nativeObj, saturation);
    }

    public void setScale(float scale) {
        setScale_0(this.nativeObj, scale);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
