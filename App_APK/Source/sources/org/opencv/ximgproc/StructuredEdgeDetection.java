package org.opencv.ximgproc;

import org.opencv.core.Algorithm;
import org.opencv.core.Mat;

public class StructuredEdgeDetection extends Algorithm {
    private static native void computeOrientation_0(long j, long j2, long j3);

    private static native void delete(long j);

    private static native void detectEdges_0(long j, long j2, long j3);

    private static native void edgesNms_0(long j, long j2, long j3, long j4, int i, int i2, float f, boolean z);

    private static native void edgesNms_1(long j, long j2, long j3, long j4, int i, int i2, float f);

    private static native void edgesNms_2(long j, long j2, long j3, long j4, int i, int i2);

    private static native void edgesNms_3(long j, long j2, long j3, long j4, int i);

    private static native void edgesNms_4(long j, long j2, long j3, long j4);

    protected StructuredEdgeDetection(long addr) {
        super(addr);
    }

    public static StructuredEdgeDetection __fromPtr__(long addr) {
        return new StructuredEdgeDetection(addr);
    }

    public void computeOrientation(Mat _src, Mat _dst) {
        computeOrientation_0(this.nativeObj, _src.nativeObj, _dst.nativeObj);
    }

    public void detectEdges(Mat _src, Mat _dst) {
        detectEdges_0(this.nativeObj, _src.nativeObj, _dst.nativeObj);
    }

    public void edgesNms(Mat edge_image, Mat orientation_image, Mat _dst, int r, int s, float m, boolean isParallel) {
        edgesNms_0(this.nativeObj, edge_image.nativeObj, orientation_image.nativeObj, _dst.nativeObj, r, s, m, isParallel);
    }

    public void edgesNms(Mat edge_image, Mat orientation_image, Mat _dst, int r, int s, float m) {
        edgesNms_1(this.nativeObj, edge_image.nativeObj, orientation_image.nativeObj, _dst.nativeObj, r, s, m);
    }

    public void edgesNms(Mat edge_image, Mat orientation_image, Mat _dst, int r, int s) {
        edgesNms_2(this.nativeObj, edge_image.nativeObj, orientation_image.nativeObj, _dst.nativeObj, r, s);
    }

    public void edgesNms(Mat edge_image, Mat orientation_image, Mat _dst, int r) {
        edgesNms_3(this.nativeObj, edge_image.nativeObj, orientation_image.nativeObj, _dst.nativeObj, r);
    }

    public void edgesNms(Mat edge_image, Mat orientation_image, Mat _dst) {
        edgesNms_4(this.nativeObj, edge_image.nativeObj, orientation_image.nativeObj, _dst.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
