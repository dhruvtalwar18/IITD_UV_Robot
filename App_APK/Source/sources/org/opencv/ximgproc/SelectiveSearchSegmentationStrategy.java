package org.opencv.ximgproc;

import org.opencv.core.Algorithm;
import org.opencv.core.Mat;

public class SelectiveSearchSegmentationStrategy extends Algorithm {
    private static native void delete(long j);

    private static native float get_0(long j, int i, int i2);

    private static native void merge_0(long j, int i, int i2);

    private static native void setImage_0(long j, long j2, long j3, long j4, int i);

    private static native void setImage_1(long j, long j2, long j3, long j4);

    protected SelectiveSearchSegmentationStrategy(long addr) {
        super(addr);
    }

    public static SelectiveSearchSegmentationStrategy __fromPtr__(long addr) {
        return new SelectiveSearchSegmentationStrategy(addr);
    }

    public float get(int r1, int r2) {
        return get_0(this.nativeObj, r1, r2);
    }

    public void merge(int r1, int r2) {
        merge_0(this.nativeObj, r1, r2);
    }

    public void setImage(Mat img, Mat regions, Mat sizes, int image_id) {
        setImage_0(this.nativeObj, img.nativeObj, regions.nativeObj, sizes.nativeObj, image_id);
    }

    public void setImage(Mat img, Mat regions, Mat sizes) {
        setImage_1(this.nativeObj, img.nativeObj, regions.nativeObj, sizes.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
