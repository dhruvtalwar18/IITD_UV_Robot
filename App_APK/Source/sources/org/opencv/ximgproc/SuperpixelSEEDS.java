package org.opencv.ximgproc;

import org.opencv.core.Algorithm;
import org.opencv.core.Mat;

public class SuperpixelSEEDS extends Algorithm {
    private static native void delete(long j);

    private static native void getLabelContourMask_0(long j, long j2, boolean z);

    private static native void getLabelContourMask_1(long j, long j2);

    private static native void getLabels_0(long j, long j2);

    private static native int getNumberOfSuperpixels_0(long j);

    private static native void iterate_0(long j, long j2, int i);

    private static native void iterate_1(long j, long j2);

    protected SuperpixelSEEDS(long addr) {
        super(addr);
    }

    public static SuperpixelSEEDS __fromPtr__(long addr) {
        return new SuperpixelSEEDS(addr);
    }

    public int getNumberOfSuperpixels() {
        return getNumberOfSuperpixels_0(this.nativeObj);
    }

    public void getLabelContourMask(Mat image, boolean thick_line) {
        getLabelContourMask_0(this.nativeObj, image.nativeObj, thick_line);
    }

    public void getLabelContourMask(Mat image) {
        getLabelContourMask_1(this.nativeObj, image.nativeObj);
    }

    public void getLabels(Mat labels_out) {
        getLabels_0(this.nativeObj, labels_out.nativeObj);
    }

    public void iterate(Mat img, int num_iterations) {
        iterate_0(this.nativeObj, img.nativeObj, num_iterations);
    }

    public void iterate(Mat img) {
        iterate_1(this.nativeObj, img.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
