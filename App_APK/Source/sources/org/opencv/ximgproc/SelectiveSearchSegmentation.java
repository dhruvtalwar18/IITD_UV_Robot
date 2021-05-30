package org.opencv.ximgproc;

import org.opencv.core.Algorithm;
import org.opencv.core.Mat;
import org.opencv.core.MatOfRect;

public class SelectiveSearchSegmentation extends Algorithm {
    private static native void addGraphSegmentation_0(long j, long j2);

    private static native void addImage_0(long j, long j2);

    private static native void addStrategy_0(long j, long j2);

    private static native void clearGraphSegmentations_0(long j);

    private static native void clearImages_0(long j);

    private static native void clearStrategies_0(long j);

    private static native void delete(long j);

    private static native void process_0(long j, long j2);

    private static native void setBaseImage_0(long j, long j2);

    private static native void switchToSelectiveSearchFast_0(long j, int i, int i2, float f);

    private static native void switchToSelectiveSearchFast_1(long j, int i, int i2);

    private static native void switchToSelectiveSearchFast_2(long j, int i);

    private static native void switchToSelectiveSearchFast_3(long j);

    private static native void switchToSelectiveSearchQuality_0(long j, int i, int i2, float f);

    private static native void switchToSelectiveSearchQuality_1(long j, int i, int i2);

    private static native void switchToSelectiveSearchQuality_2(long j, int i);

    private static native void switchToSelectiveSearchQuality_3(long j);

    private static native void switchToSingleStrategy_0(long j, int i, float f);

    private static native void switchToSingleStrategy_1(long j, int i);

    private static native void switchToSingleStrategy_2(long j);

    protected SelectiveSearchSegmentation(long addr) {
        super(addr);
    }

    public static SelectiveSearchSegmentation __fromPtr__(long addr) {
        return new SelectiveSearchSegmentation(addr);
    }

    public void addGraphSegmentation(GraphSegmentation g) {
        addGraphSegmentation_0(this.nativeObj, g.getNativeObjAddr());
    }

    public void addImage(Mat img) {
        addImage_0(this.nativeObj, img.nativeObj);
    }

    public void addStrategy(SelectiveSearchSegmentationStrategy s) {
        addStrategy_0(this.nativeObj, s.getNativeObjAddr());
    }

    public void clearGraphSegmentations() {
        clearGraphSegmentations_0(this.nativeObj);
    }

    public void clearImages() {
        clearImages_0(this.nativeObj);
    }

    public void clearStrategies() {
        clearStrategies_0(this.nativeObj);
    }

    public void process(MatOfRect rects) {
        process_0(this.nativeObj, rects.nativeObj);
    }

    public void setBaseImage(Mat img) {
        setBaseImage_0(this.nativeObj, img.nativeObj);
    }

    public void switchToSelectiveSearchFast(int base_k, int inc_k, float sigma) {
        switchToSelectiveSearchFast_0(this.nativeObj, base_k, inc_k, sigma);
    }

    public void switchToSelectiveSearchFast(int base_k, int inc_k) {
        switchToSelectiveSearchFast_1(this.nativeObj, base_k, inc_k);
    }

    public void switchToSelectiveSearchFast(int base_k) {
        switchToSelectiveSearchFast_2(this.nativeObj, base_k);
    }

    public void switchToSelectiveSearchFast() {
        switchToSelectiveSearchFast_3(this.nativeObj);
    }

    public void switchToSelectiveSearchQuality(int base_k, int inc_k, float sigma) {
        switchToSelectiveSearchQuality_0(this.nativeObj, base_k, inc_k, sigma);
    }

    public void switchToSelectiveSearchQuality(int base_k, int inc_k) {
        switchToSelectiveSearchQuality_1(this.nativeObj, base_k, inc_k);
    }

    public void switchToSelectiveSearchQuality(int base_k) {
        switchToSelectiveSearchQuality_2(this.nativeObj, base_k);
    }

    public void switchToSelectiveSearchQuality() {
        switchToSelectiveSearchQuality_3(this.nativeObj);
    }

    public void switchToSingleStrategy(int k, float sigma) {
        switchToSingleStrategy_0(this.nativeObj, k, sigma);
    }

    public void switchToSingleStrategy(int k) {
        switchToSingleStrategy_1(this.nativeObj, k);
    }

    public void switchToSingleStrategy() {
        switchToSingleStrategy_2(this.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
