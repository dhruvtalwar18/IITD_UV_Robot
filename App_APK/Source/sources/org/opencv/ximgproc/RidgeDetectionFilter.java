package org.opencv.ximgproc;

import org.opencv.core.Algorithm;
import org.opencv.core.Mat;

public class RidgeDetectionFilter extends Algorithm {
    private static native long create_0(int i, int i2, int i3, int i4, int i5, double d, double d2, int i6);

    private static native long create_1(int i, int i2, int i3, int i4, int i5, double d, double d2);

    private static native long create_2(int i, int i2, int i3, int i4, int i5, double d);

    private static native long create_3(int i, int i2, int i3, int i4, int i5);

    private static native long create_4(int i, int i2, int i3, int i4);

    private static native long create_5(int i, int i2, int i3);

    private static native long create_6(int i, int i2);

    private static native long create_7(int i);

    private static native long create_8();

    private static native void delete(long j);

    private static native void getRidgeFilteredImage_0(long j, long j2, long j3);

    protected RidgeDetectionFilter(long addr) {
        super(addr);
    }

    public static RidgeDetectionFilter __fromPtr__(long addr) {
        return new RidgeDetectionFilter(addr);
    }

    public static RidgeDetectionFilter create(int ddepth, int dx, int dy, int ksize, int out_dtype, double scale, double delta, int borderType) {
        return __fromPtr__(create_0(ddepth, dx, dy, ksize, out_dtype, scale, delta, borderType));
    }

    public static RidgeDetectionFilter create(int ddepth, int dx, int dy, int ksize, int out_dtype, double scale, double delta) {
        return __fromPtr__(create_1(ddepth, dx, dy, ksize, out_dtype, scale, delta));
    }

    public static RidgeDetectionFilter create(int ddepth, int dx, int dy, int ksize, int out_dtype, double scale) {
        return __fromPtr__(create_2(ddepth, dx, dy, ksize, out_dtype, scale));
    }

    public static RidgeDetectionFilter create(int ddepth, int dx, int dy, int ksize, int out_dtype) {
        return __fromPtr__(create_3(ddepth, dx, dy, ksize, out_dtype));
    }

    public static RidgeDetectionFilter create(int ddepth, int dx, int dy, int ksize) {
        return __fromPtr__(create_4(ddepth, dx, dy, ksize));
    }

    public static RidgeDetectionFilter create(int ddepth, int dx, int dy) {
        return __fromPtr__(create_5(ddepth, dx, dy));
    }

    public static RidgeDetectionFilter create(int ddepth, int dx) {
        return __fromPtr__(create_6(ddepth, dx));
    }

    public static RidgeDetectionFilter create(int ddepth) {
        return __fromPtr__(create_7(ddepth));
    }

    public static RidgeDetectionFilter create() {
        return __fromPtr__(create_8());
    }

    public void getRidgeFilteredImage(Mat _img, Mat out) {
        getRidgeFilteredImage_0(this.nativeObj, _img.nativeObj, out.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
