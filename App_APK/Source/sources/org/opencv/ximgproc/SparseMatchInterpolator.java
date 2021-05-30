package org.opencv.ximgproc;

import org.opencv.core.Algorithm;
import org.opencv.core.Mat;

public class SparseMatchInterpolator extends Algorithm {
    private static native void delete(long j);

    private static native void interpolate_0(long j, long j2, long j3, long j4, long j5, long j6);

    protected SparseMatchInterpolator(long addr) {
        super(addr);
    }

    public static SparseMatchInterpolator __fromPtr__(long addr) {
        return new SparseMatchInterpolator(addr);
    }

    public void interpolate(Mat from_image, Mat from_points, Mat to_image, Mat to_points, Mat dense_flow) {
        interpolate_0(this.nativeObj, from_image.nativeObj, from_points.nativeObj, to_image.nativeObj, to_points.nativeObj, dense_flow.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
