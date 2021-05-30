package org.opencv.xfeatures2d;

import org.opencv.features2d.Feature2D;

public class VGG extends Feature2D {
    private static native long create_0(int i, float f, boolean z, boolean z2, float f2, boolean z3);

    private static native long create_1(int i, float f, boolean z, boolean z2, float f2);

    private static native long create_2(int i, float f, boolean z, boolean z2);

    private static native long create_3(int i, float f, boolean z);

    private static native long create_4(int i, float f);

    private static native long create_5(int i);

    private static native long create_6();

    private static native void delete(long j);

    private static native float getScaleFactor_0(long j);

    private static native float getSigma_0(long j);

    private static native boolean getUseNormalizeDescriptor_0(long j);

    private static native boolean getUseNormalizeImage_0(long j);

    private static native boolean getUseScaleOrientation_0(long j);

    private static native void setScaleFactor_0(long j, float f);

    private static native void setSigma_0(long j, float f);

    private static native void setUseNormalizeDescriptor_0(long j, boolean z);

    private static native void setUseNormalizeImage_0(long j, boolean z);

    private static native void setUseScaleOrientation_0(long j, boolean z);

    protected VGG(long addr) {
        super(addr);
    }

    public static VGG __fromPtr__(long addr) {
        return new VGG(addr);
    }

    public static VGG create(int desc, float isigma, boolean img_normalize, boolean use_scale_orientation, float scale_factor, boolean dsc_normalize) {
        return __fromPtr__(create_0(desc, isigma, img_normalize, use_scale_orientation, scale_factor, dsc_normalize));
    }

    public static VGG create(int desc, float isigma, boolean img_normalize, boolean use_scale_orientation, float scale_factor) {
        return __fromPtr__(create_1(desc, isigma, img_normalize, use_scale_orientation, scale_factor));
    }

    public static VGG create(int desc, float isigma, boolean img_normalize, boolean use_scale_orientation) {
        return __fromPtr__(create_2(desc, isigma, img_normalize, use_scale_orientation));
    }

    public static VGG create(int desc, float isigma, boolean img_normalize) {
        return __fromPtr__(create_3(desc, isigma, img_normalize));
    }

    public static VGG create(int desc, float isigma) {
        return __fromPtr__(create_4(desc, isigma));
    }

    public static VGG create(int desc) {
        return __fromPtr__(create_5(desc));
    }

    public static VGG create() {
        return __fromPtr__(create_6());
    }

    public boolean getUseNormalizeDescriptor() {
        return getUseNormalizeDescriptor_0(this.nativeObj);
    }

    public boolean getUseNormalizeImage() {
        return getUseNormalizeImage_0(this.nativeObj);
    }

    public boolean getUseScaleOrientation() {
        return getUseScaleOrientation_0(this.nativeObj);
    }

    public float getScaleFactor() {
        return getScaleFactor_0(this.nativeObj);
    }

    public float getSigma() {
        return getSigma_0(this.nativeObj);
    }

    public void setScaleFactor(float scale_factor) {
        setScaleFactor_0(this.nativeObj, scale_factor);
    }

    public void setSigma(float isigma) {
        setSigma_0(this.nativeObj, isigma);
    }

    public void setUseNormalizeDescriptor(boolean dsc_normalize) {
        setUseNormalizeDescriptor_0(this.nativeObj, dsc_normalize);
    }

    public void setUseNormalizeImage(boolean img_normalize) {
        setUseNormalizeImage_0(this.nativeObj, img_normalize);
    }

    public void setUseScaleOrientation(boolean use_scale_orientation) {
        setUseScaleOrientation_0(this.nativeObj, use_scale_orientation);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
