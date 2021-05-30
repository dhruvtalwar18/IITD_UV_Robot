package org.opencv.ximgproc;

import org.opencv.core.Algorithm;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

public class DisparityFilter extends Algorithm {
    private static native void delete(long j);

    private static native void filter_0(long j, long j2, long j3, long j4, long j5, int i, int i2, int i3, int i4, long j6);

    private static native void filter_1(long j, long j2, long j3, long j4, long j5, int i, int i2, int i3, int i4);

    private static native void filter_2(long j, long j2, long j3, long j4, long j5);

    private static native void filter_3(long j, long j2, long j3, long j4);

    protected DisparityFilter(long addr) {
        super(addr);
    }

    public static DisparityFilter __fromPtr__(long addr) {
        return new DisparityFilter(addr);
    }

    public void filter(Mat disparity_map_left, Mat left_view, Mat filtered_disparity_map, Mat disparity_map_right, Rect ROI, Mat right_view) {
        Rect rect = ROI;
        long j = this.nativeObj;
        long j2 = disparity_map_left.nativeObj;
        long j3 = left_view.nativeObj;
        long j4 = filtered_disparity_map.nativeObj;
        long j5 = disparity_map_right.nativeObj;
        int i = rect.x;
        int i2 = rect.y;
        int i3 = rect.width;
        int i4 = i3;
        filter_0(j, j2, j3, j4, j5, i, i2, i4, rect.height, right_view.nativeObj);
    }

    public void filter(Mat disparity_map_left, Mat left_view, Mat filtered_disparity_map, Mat disparity_map_right, Rect ROI) {
        Rect rect = ROI;
        filter_1(this.nativeObj, disparity_map_left.nativeObj, left_view.nativeObj, filtered_disparity_map.nativeObj, disparity_map_right.nativeObj, rect.x, rect.y, rect.width, rect.height);
    }

    public void filter(Mat disparity_map_left, Mat left_view, Mat filtered_disparity_map, Mat disparity_map_right) {
        filter_2(this.nativeObj, disparity_map_left.nativeObj, left_view.nativeObj, filtered_disparity_map.nativeObj, disparity_map_right.nativeObj);
    }

    public void filter(Mat disparity_map_left, Mat left_view, Mat filtered_disparity_map) {
        filter_3(this.nativeObj, disparity_map_left.nativeObj, left_view.nativeObj, filtered_disparity_map.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
