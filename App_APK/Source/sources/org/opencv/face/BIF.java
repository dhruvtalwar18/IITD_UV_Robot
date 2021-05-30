package org.opencv.face;

import org.opencv.core.Algorithm;
import org.opencv.core.Mat;

public class BIF extends Algorithm {
    private static native void compute_0(long j, long j2, long j3);

    private static native long create_0(int i, int i2);

    private static native long create_1(int i);

    private static native long create_2();

    private static native void delete(long j);

    private static native int getNumBands_0(long j);

    private static native int getNumRotations_0(long j);

    protected BIF(long addr) {
        super(addr);
    }

    public static BIF __fromPtr__(long addr) {
        return new BIF(addr);
    }

    public static BIF create(int num_bands, int num_rotations) {
        return __fromPtr__(create_0(num_bands, num_rotations));
    }

    public static BIF create(int num_bands) {
        return __fromPtr__(create_1(num_bands));
    }

    public static BIF create() {
        return __fromPtr__(create_2());
    }

    public int getNumBands() {
        return getNumBands_0(this.nativeObj);
    }

    public int getNumRotations() {
        return getNumRotations_0(this.nativeObj);
    }

    public void compute(Mat image, Mat features) {
        compute_0(this.nativeObj, image.nativeObj, features.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
