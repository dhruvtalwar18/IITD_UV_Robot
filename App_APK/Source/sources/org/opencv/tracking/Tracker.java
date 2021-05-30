package org.opencv.tracking;

import org.opencv.core.Algorithm;
import org.opencv.core.Mat;
import org.opencv.core.Rect2d;

public class Tracker extends Algorithm {
    private static native void delete(long j);

    private static native boolean init_0(long j, long j2, double d, double d2, double d3, double d4);

    private static native boolean update_0(long j, long j2, double[] dArr);

    protected Tracker(long addr) {
        super(addr);
    }

    public static Tracker __fromPtr__(long addr) {
        return new Tracker(addr);
    }

    public boolean init(Mat image, Rect2d boundingBox) {
        return init_0(this.nativeObj, image.nativeObj, boundingBox.x, boundingBox.y, boundingBox.width, boundingBox.height);
    }

    public boolean update(Mat image, Rect2d boundingBox) {
        double[] boundingBox_out = new double[4];
        boolean retVal = update_0(this.nativeObj, image.nativeObj, boundingBox_out);
        if (boundingBox != null) {
            boundingBox.x = boundingBox_out[0];
            boundingBox.y = boundingBox_out[1];
            boundingBox.width = boundingBox_out[2];
            boundingBox.height = boundingBox_out[3];
        }
        return retVal;
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
