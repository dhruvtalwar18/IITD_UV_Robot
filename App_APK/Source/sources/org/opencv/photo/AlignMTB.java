package org.opencv.photo;

import java.util.List;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.utils.Converters;

public class AlignMTB extends AlignExposures {
    private static native double[] calculateShift_0(long j, long j2, long j3);

    private static native void computeBitmaps_0(long j, long j2, long j3, long j4);

    private static native void delete(long j);

    private static native boolean getCut_0(long j);

    private static native int getExcludeRange_0(long j);

    private static native int getMaxBits_0(long j);

    private static native void process_0(long j, long j2, long j3, long j4, long j5);

    private static native void process_1(long j, long j2, long j3);

    private static native void setCut_0(long j, boolean z);

    private static native void setExcludeRange_0(long j, int i);

    private static native void setMaxBits_0(long j, int i);

    private static native void shiftMat_0(long j, long j2, long j3, double d, double d2);

    protected AlignMTB(long addr) {
        super(addr);
    }

    public static AlignMTB __fromPtr__(long addr) {
        return new AlignMTB(addr);
    }

    public Point calculateShift(Mat img0, Mat img1) {
        return new Point(calculateShift_0(this.nativeObj, img0.nativeObj, img1.nativeObj));
    }

    public boolean getCut() {
        return getCut_0(this.nativeObj);
    }

    public int getExcludeRange() {
        return getExcludeRange_0(this.nativeObj);
    }

    public int getMaxBits() {
        return getMaxBits_0(this.nativeObj);
    }

    public void computeBitmaps(Mat img, Mat tb, Mat eb) {
        computeBitmaps_0(this.nativeObj, img.nativeObj, tb.nativeObj, eb.nativeObj);
    }

    public void process(List<Mat> src, List<Mat> dst, Mat times, Mat response) {
        process_0(this.nativeObj, Converters.vector_Mat_to_Mat(src).nativeObj, Converters.vector_Mat_to_Mat(dst).nativeObj, times.nativeObj, response.nativeObj);
    }

    public void process(List<Mat> src, List<Mat> dst) {
        process_1(this.nativeObj, Converters.vector_Mat_to_Mat(src).nativeObj, Converters.vector_Mat_to_Mat(dst).nativeObj);
    }

    public void setCut(boolean value) {
        setCut_0(this.nativeObj, value);
    }

    public void setExcludeRange(int exclude_range) {
        setExcludeRange_0(this.nativeObj, exclude_range);
    }

    public void setMaxBits(int max_bits) {
        setMaxBits_0(this.nativeObj, max_bits);
    }

    public void shiftMat(Mat src, Mat dst, Point shift) {
        shiftMat_0(this.nativeObj, src.nativeObj, dst.nativeObj, shift.x, shift.y);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
