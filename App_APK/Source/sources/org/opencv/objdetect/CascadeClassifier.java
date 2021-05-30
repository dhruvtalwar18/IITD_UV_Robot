package org.opencv.objdetect;

import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfRect;
import org.opencv.core.Size;

public class CascadeClassifier {
    protected final long nativeObj;

    private static native long CascadeClassifier_0(String str);

    private static native long CascadeClassifier_1();

    private static native boolean convert_0(String str, String str2);

    private static native void delete(long j);

    private static native void detectMultiScale2_0(long j, long j2, long j3, long j4, double d, int i, int i2, double d2, double d3, double d4, double d5);

    private static native void detectMultiScale2_1(long j, long j2, long j3, long j4, double d, int i, int i2, double d2, double d3);

    private static native void detectMultiScale2_2(long j, long j2, long j3, long j4, double d, int i, int i2);

    private static native void detectMultiScale2_3(long j, long j2, long j3, long j4, double d, int i);

    private static native void detectMultiScale2_4(long j, long j2, long j3, long j4, double d);

    private static native void detectMultiScale2_5(long j, long j2, long j3, long j4);

    private static native void detectMultiScale3_0(long j, long j2, long j3, long j4, long j5, double d, int i, int i2, double d2, double d3, double d4, double d5, boolean z);

    private static native void detectMultiScale3_1(long j, long j2, long j3, long j4, long j5, double d, int i, int i2, double d2, double d3, double d4, double d5);

    private static native void detectMultiScale3_2(long j, long j2, long j3, long j4, long j5, double d, int i, int i2, double d2, double d3);

    private static native void detectMultiScale3_3(long j, long j2, long j3, long j4, long j5, double d, int i, int i2);

    private static native void detectMultiScale3_4(long j, long j2, long j3, long j4, long j5, double d, int i);

    private static native void detectMultiScale3_5(long j, long j2, long j3, long j4, long j5, double d);

    private static native void detectMultiScale3_6(long j, long j2, long j3, long j4, long j5);

    private static native void detectMultiScale_0(long j, long j2, long j3, double d, int i, int i2, double d2, double d3, double d4, double d5);

    private static native void detectMultiScale_1(long j, long j2, long j3, double d, int i, int i2, double d2, double d3);

    private static native void detectMultiScale_2(long j, long j2, long j3, double d, int i, int i2);

    private static native void detectMultiScale_3(long j, long j2, long j3, double d, int i);

    private static native void detectMultiScale_4(long j, long j2, long j3, double d);

    private static native void detectMultiScale_5(long j, long j2, long j3);

    private static native boolean empty_0(long j);

    private static native int getFeatureType_0(long j);

    private static native double[] getOriginalWindowSize_0(long j);

    private static native boolean isOldFormatCascade_0(long j);

    private static native boolean load_0(long j, String str);

    protected CascadeClassifier(long addr) {
        this.nativeObj = addr;
    }

    public long getNativeObjAddr() {
        return this.nativeObj;
    }

    public static CascadeClassifier __fromPtr__(long addr) {
        return new CascadeClassifier(addr);
    }

    public CascadeClassifier(String filename) {
        this.nativeObj = CascadeClassifier_0(filename);
    }

    public CascadeClassifier() {
        this.nativeObj = CascadeClassifier_1();
    }

    public Size getOriginalWindowSize() {
        return new Size(getOriginalWindowSize_0(this.nativeObj));
    }

    public static boolean convert(String oldcascade, String newcascade) {
        return convert_0(oldcascade, newcascade);
    }

    public boolean empty() {
        return empty_0(this.nativeObj);
    }

    public boolean isOldFormatCascade() {
        return isOldFormatCascade_0(this.nativeObj);
    }

    public boolean load(String filename) {
        return load_0(this.nativeObj, filename);
    }

    public int getFeatureType() {
        return getFeatureType_0(this.nativeObj);
    }

    public void detectMultiScale(Mat image, MatOfRect objects, double scaleFactor, int minNeighbors, int flags, Size minSize, Size maxSize) {
        Size size = minSize;
        Size size2 = maxSize;
        MatOfRect matOfRect = objects;
        long j = this.nativeObj;
        MatOfRect matOfRect2 = matOfRect;
        long j2 = j;
        detectMultiScale_0(j2, image.nativeObj, matOfRect.nativeObj, scaleFactor, minNeighbors, flags, size.width, size.height, size2.width, size2.height);
    }

    public void detectMultiScale(Mat image, MatOfRect objects, double scaleFactor, int minNeighbors, int flags, Size minSize) {
        Size size = minSize;
        detectMultiScale_1(this.nativeObj, image.nativeObj, objects.nativeObj, scaleFactor, minNeighbors, flags, size.width, size.height);
    }

    public void detectMultiScale(Mat image, MatOfRect objects, double scaleFactor, int minNeighbors, int flags) {
        detectMultiScale_2(this.nativeObj, image.nativeObj, objects.nativeObj, scaleFactor, minNeighbors, flags);
    }

    public void detectMultiScale(Mat image, MatOfRect objects, double scaleFactor, int minNeighbors) {
        detectMultiScale_3(this.nativeObj, image.nativeObj, objects.nativeObj, scaleFactor, minNeighbors);
    }

    public void detectMultiScale(Mat image, MatOfRect objects, double scaleFactor) {
        detectMultiScale_4(this.nativeObj, image.nativeObj, objects.nativeObj, scaleFactor);
    }

    public void detectMultiScale(Mat image, MatOfRect objects) {
        detectMultiScale_5(this.nativeObj, image.nativeObj, objects.nativeObj);
    }

    public void detectMultiScale2(Mat image, MatOfRect objects, MatOfInt numDetections, double scaleFactor, int minNeighbors, int flags, Size minSize, Size maxSize) {
        Size size = minSize;
        Size size2 = maxSize;
        MatOfRect matOfRect = objects;
        MatOfInt matOfInt = numDetections;
        long j = this.nativeObj;
        MatOfRect matOfRect2 = matOfRect;
        MatOfInt matOfInt2 = matOfInt;
        long j2 = j;
        detectMultiScale2_0(j2, image.nativeObj, matOfRect.nativeObj, matOfInt.nativeObj, scaleFactor, minNeighbors, flags, size.width, size.height, size2.width, size2.height);
    }

    public void detectMultiScale2(Mat image, MatOfRect objects, MatOfInt numDetections, double scaleFactor, int minNeighbors, int flags, Size minSize) {
        Size size = minSize;
        MatOfRect matOfRect = objects;
        MatOfInt matOfInt = numDetections;
        MatOfRect matOfRect2 = matOfRect;
        MatOfInt matOfInt2 = matOfInt;
        detectMultiScale2_1(this.nativeObj, image.nativeObj, matOfRect.nativeObj, matOfInt.nativeObj, scaleFactor, minNeighbors, flags, size.width, size.height);
    }

    public void detectMultiScale2(Mat image, MatOfRect objects, MatOfInt numDetections, double scaleFactor, int minNeighbors, int flags) {
        detectMultiScale2_2(this.nativeObj, image.nativeObj, objects.nativeObj, numDetections.nativeObj, scaleFactor, minNeighbors, flags);
    }

    public void detectMultiScale2(Mat image, MatOfRect objects, MatOfInt numDetections, double scaleFactor, int minNeighbors) {
        detectMultiScale2_3(this.nativeObj, image.nativeObj, objects.nativeObj, numDetections.nativeObj, scaleFactor, minNeighbors);
    }

    public void detectMultiScale2(Mat image, MatOfRect objects, MatOfInt numDetections, double scaleFactor) {
        detectMultiScale2_4(this.nativeObj, image.nativeObj, objects.nativeObj, numDetections.nativeObj, scaleFactor);
    }

    public void detectMultiScale2(Mat image, MatOfRect objects, MatOfInt numDetections) {
        detectMultiScale2_5(this.nativeObj, image.nativeObj, objects.nativeObj, numDetections.nativeObj);
    }

    public void detectMultiScale3(Mat image, MatOfRect objects, MatOfInt rejectLevels, MatOfDouble levelWeights, double scaleFactor, int minNeighbors, int flags, Size minSize, Size maxSize, boolean outputRejectLevels) {
        Size size = minSize;
        Size size2 = maxSize;
        MatOfRect matOfRect = objects;
        Mat mat = rejectLevels;
        long j = this.nativeObj;
        long j2 = j;
        MatOfDouble matOfDouble = levelWeights;
        MatOfRect matOfRect2 = matOfRect;
        MatOfRect matOfRect3 = matOfRect2;
        Mat objects_mat = mat;
        MatOfDouble matOfDouble2 = matOfDouble;
        long j3 = j2;
        MatOfDouble matOfDouble3 = matOfDouble2;
        detectMultiScale3_0(j3, image.nativeObj, matOfRect.nativeObj, mat.nativeObj, matOfDouble.nativeObj, scaleFactor, minNeighbors, flags, size.width, size.height, size2.width, size2.height, outputRejectLevels);
    }

    public void detectMultiScale3(Mat image, MatOfRect objects, MatOfInt rejectLevels, MatOfDouble levelWeights, double scaleFactor, int minNeighbors, int flags, Size minSize, Size maxSize) {
        Size size = minSize;
        Size size2 = maxSize;
        MatOfRect matOfRect = objects;
        Mat mat = rejectLevels;
        long j = this.nativeObj;
        long j2 = j;
        MatOfDouble matOfDouble = levelWeights;
        MatOfRect matOfRect2 = matOfRect;
        MatOfRect matOfRect3 = matOfRect2;
        Mat objects_mat = mat;
        MatOfDouble matOfDouble2 = matOfDouble;
        long j3 = j2;
        MatOfDouble matOfDouble3 = matOfDouble2;
        detectMultiScale3_1(j3, image.nativeObj, matOfRect.nativeObj, mat.nativeObj, matOfDouble.nativeObj, scaleFactor, minNeighbors, flags, size.width, size.height, size2.width, size2.height);
    }

    public void detectMultiScale3(Mat image, MatOfRect objects, MatOfInt rejectLevels, MatOfDouble levelWeights, double scaleFactor, int minNeighbors, int flags, Size minSize) {
        Size size = minSize;
        MatOfRect matOfRect = objects;
        MatOfInt matOfInt = rejectLevels;
        MatOfDouble matOfDouble = levelWeights;
        long j = this.nativeObj;
        MatOfInt matOfInt2 = matOfInt;
        MatOfDouble matOfDouble2 = matOfDouble;
        MatOfRect matOfRect2 = matOfRect;
        long j2 = j;
        detectMultiScale3_2(j2, image.nativeObj, matOfRect.nativeObj, matOfInt.nativeObj, matOfDouble.nativeObj, scaleFactor, minNeighbors, flags, size.width, size.height);
    }

    public void detectMultiScale3(Mat image, MatOfRect objects, MatOfInt rejectLevels, MatOfDouble levelWeights, double scaleFactor, int minNeighbors, int flags) {
        detectMultiScale3_3(this.nativeObj, image.nativeObj, objects.nativeObj, rejectLevels.nativeObj, levelWeights.nativeObj, scaleFactor, minNeighbors, flags);
    }

    public void detectMultiScale3(Mat image, MatOfRect objects, MatOfInt rejectLevels, MatOfDouble levelWeights, double scaleFactor, int minNeighbors) {
        detectMultiScale3_4(this.nativeObj, image.nativeObj, objects.nativeObj, rejectLevels.nativeObj, levelWeights.nativeObj, scaleFactor, minNeighbors);
    }

    public void detectMultiScale3(Mat image, MatOfRect objects, MatOfInt rejectLevels, MatOfDouble levelWeights, double scaleFactor) {
        detectMultiScale3_5(this.nativeObj, image.nativeObj, objects.nativeObj, rejectLevels.nativeObj, levelWeights.nativeObj, scaleFactor);
    }

    public void detectMultiScale3(Mat image, MatOfRect objects, MatOfInt rejectLevels, MatOfDouble levelWeights) {
        detectMultiScale3_6(this.nativeObj, image.nativeObj, objects.nativeObj, rejectLevels.nativeObj, levelWeights.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
