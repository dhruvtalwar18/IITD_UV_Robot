package org.opencv.ximgproc;

import org.opencv.core.Algorithm;
import org.opencv.core.Mat;

public class SuperpixelSLIC extends Algorithm {
    private static native void delete(long j);

    private static native void enforceLabelConnectivity_0(long j, int i);

    private static native void enforceLabelConnectivity_1(long j);

    private static native void getLabelContourMask_0(long j, long j2, boolean z);

    private static native void getLabelContourMask_1(long j, long j2);

    private static native void getLabels_0(long j, long j2);

    private static native int getNumberOfSuperpixels_0(long j);

    private static native void iterate_0(long j, int i);

    private static native void iterate_1(long j);

    protected SuperpixelSLIC(long addr) {
        super(addr);
    }

    public static SuperpixelSLIC __fromPtr__(long addr) {
        return new SuperpixelSLIC(addr);
    }

    public int getNumberOfSuperpixels() {
        return getNumberOfSuperpixels_0(this.nativeObj);
    }

    public void enforceLabelConnectivity(int min_element_size) {
        enforceLabelConnectivity_0(this.nativeObj, min_element_size);
    }

    public void enforceLabelConnectivity() {
        enforceLabelConnectivity_1(this.nativeObj);
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

    public void iterate(int num_iterations) {
        iterate_0(this.nativeObj, num_iterations);
    }

    public void iterate() {
        iterate_1(this.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
