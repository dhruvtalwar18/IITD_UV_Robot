package org.opencv.face;

import java.util.ArrayList;
import java.util.List;
import org.opencv.core.Mat;
import org.opencv.utils.Converters;

public class LBPHFaceRecognizer extends FaceRecognizer {
    private static native long create_0(int i, int i2, int i3, int i4, double d);

    private static native long create_1(int i, int i2, int i3, int i4);

    private static native long create_2(int i, int i2, int i3);

    private static native long create_3(int i, int i2);

    private static native long create_4(int i);

    private static native long create_5();

    private static native void delete(long j);

    private static native int getGridX_0(long j);

    private static native int getGridY_0(long j);

    private static native long getHistograms_0(long j);

    private static native long getLabels_0(long j);

    private static native int getNeighbors_0(long j);

    private static native int getRadius_0(long j);

    private static native double getThreshold_0(long j);

    private static native void setGridX_0(long j, int i);

    private static native void setGridY_0(long j, int i);

    private static native void setNeighbors_0(long j, int i);

    private static native void setRadius_0(long j, int i);

    private static native void setThreshold_0(long j, double d);

    protected LBPHFaceRecognizer(long addr) {
        super(addr);
    }

    public static LBPHFaceRecognizer __fromPtr__(long addr) {
        return new LBPHFaceRecognizer(addr);
    }

    public Mat getLabels() {
        return new Mat(getLabels_0(this.nativeObj));
    }

    public static LBPHFaceRecognizer create(int radius, int neighbors, int grid_x, int grid_y, double threshold) {
        return __fromPtr__(create_0(radius, neighbors, grid_x, grid_y, threshold));
    }

    public static LBPHFaceRecognizer create(int radius, int neighbors, int grid_x, int grid_y) {
        return __fromPtr__(create_1(radius, neighbors, grid_x, grid_y));
    }

    public static LBPHFaceRecognizer create(int radius, int neighbors, int grid_x) {
        return __fromPtr__(create_2(radius, neighbors, grid_x));
    }

    public static LBPHFaceRecognizer create(int radius, int neighbors) {
        return __fromPtr__(create_3(radius, neighbors));
    }

    public static LBPHFaceRecognizer create(int radius) {
        return __fromPtr__(create_4(radius));
    }

    public static LBPHFaceRecognizer create() {
        return __fromPtr__(create_5());
    }

    public double getThreshold() {
        return getThreshold_0(this.nativeObj);
    }

    public int getGridX() {
        return getGridX_0(this.nativeObj);
    }

    public int getGridY() {
        return getGridY_0(this.nativeObj);
    }

    public int getNeighbors() {
        return getNeighbors_0(this.nativeObj);
    }

    public int getRadius() {
        return getRadius_0(this.nativeObj);
    }

    public List<Mat> getHistograms() {
        List<Mat> retVal = new ArrayList<>();
        Converters.Mat_to_vector_Mat(new Mat(getHistograms_0(this.nativeObj)), retVal);
        return retVal;
    }

    public void setGridX(int val) {
        setGridX_0(this.nativeObj, val);
    }

    public void setGridY(int val) {
        setGridY_0(this.nativeObj, val);
    }

    public void setNeighbors(int val) {
        setNeighbors_0(this.nativeObj, val);
    }

    public void setRadius(int val) {
        setRadius_0(this.nativeObj, val);
    }

    public void setThreshold(double val) {
        setThreshold_0(this.nativeObj, val);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
