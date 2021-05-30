package org.opencv.xfeatures2d;

import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Size;

public class Xfeatures2d {
    public static final int SURF_CUDA_ANGLE_ROW = 5;
    public static final int SURF_CUDA_HESSIAN_ROW = 6;
    public static final int SURF_CUDA_LAPLACIAN_ROW = 2;
    public static final int SURF_CUDA_OCTAVE_ROW = 3;
    public static final int SURF_CUDA_ROWS_COUNT = 7;
    public static final int SURF_CUDA_SIZE_ROW = 4;
    public static final int SURF_CUDA_X_ROW = 0;
    public static final int SURF_CUDA_Y_ROW = 1;

    private static native void matchGMS_0(double d, double d2, double d3, double d4, long j, long j2, long j3, long j4, boolean z, boolean z2, double d5);

    private static native void matchGMS_1(double d, double d2, double d3, double d4, long j, long j2, long j3, long j4, boolean z, boolean z2);

    private static native void matchGMS_2(double d, double d2, double d3, double d4, long j, long j2, long j3, long j4, boolean z);

    private static native void matchGMS_3(double d, double d2, double d3, double d4, long j, long j2, long j3, long j4);

    public static void matchGMS(Size size1, Size size2, MatOfKeyPoint keypoints1, MatOfKeyPoint keypoints2, MatOfDMatch matches1to2, MatOfDMatch matchesGMS, boolean withRotation, boolean withScale, double thresholdFactor) {
        Size size = size1;
        Size size3 = size2;
        MatOfKeyPoint matOfKeyPoint = keypoints1;
        MatOfKeyPoint matOfKeyPoint2 = keypoints2;
        double d = size.width;
        double d2 = d;
        MatOfDMatch matOfDMatch = matches1to2;
        MatOfDMatch matOfDMatch2 = matchesGMS;
        MatOfKeyPoint matOfKeyPoint3 = matOfKeyPoint;
        MatOfKeyPoint matOfKeyPoint4 = matOfKeyPoint2;
        MatOfDMatch matOfDMatch3 = matOfDMatch;
        MatOfDMatch matOfDMatch4 = matOfDMatch2;
        double d3 = d2;
        matchGMS_0(d3, size.height, size3.width, size3.height, matOfKeyPoint.nativeObj, matOfKeyPoint2.nativeObj, matOfDMatch.nativeObj, matOfDMatch2.nativeObj, withRotation, withScale, thresholdFactor);
    }

    public static void matchGMS(Size size1, Size size2, MatOfKeyPoint keypoints1, MatOfKeyPoint keypoints2, MatOfDMatch matches1to2, MatOfDMatch matchesGMS, boolean withRotation, boolean withScale) {
        Size size = size1;
        Size size3 = size2;
        MatOfKeyPoint matOfKeyPoint = keypoints1;
        MatOfKeyPoint matOfKeyPoint2 = keypoints2;
        double d = size.width;
        double d2 = d;
        MatOfDMatch matOfDMatch = matches1to2;
        MatOfDMatch matOfDMatch2 = matchesGMS;
        MatOfKeyPoint matOfKeyPoint3 = matOfKeyPoint;
        MatOfKeyPoint matOfKeyPoint4 = matOfKeyPoint2;
        MatOfDMatch matOfDMatch3 = matOfDMatch;
        MatOfDMatch matOfDMatch4 = matOfDMatch2;
        double d3 = d2;
        matchGMS_1(d3, size.height, size3.width, size3.height, matOfKeyPoint.nativeObj, matOfKeyPoint2.nativeObj, matOfDMatch.nativeObj, matOfDMatch2.nativeObj, withRotation, withScale);
    }

    public static void matchGMS(Size size1, Size size2, MatOfKeyPoint keypoints1, MatOfKeyPoint keypoints2, MatOfDMatch matches1to2, MatOfDMatch matchesGMS, boolean withRotation) {
        Size size = size1;
        Size size3 = size2;
        MatOfKeyPoint matOfKeyPoint = keypoints1;
        MatOfKeyPoint matOfKeyPoint2 = keypoints2;
        double d = size.width;
        double d2 = d;
        MatOfDMatch matOfDMatch = matches1to2;
        MatOfDMatch matOfDMatch2 = matchesGMS;
        MatOfKeyPoint matOfKeyPoint3 = matOfKeyPoint;
        MatOfKeyPoint matOfKeyPoint4 = matOfKeyPoint2;
        MatOfDMatch matOfDMatch3 = matOfDMatch;
        MatOfDMatch matOfDMatch4 = matOfDMatch2;
        double d3 = d2;
        matchGMS_2(d3, size.height, size3.width, size3.height, matOfKeyPoint.nativeObj, matOfKeyPoint2.nativeObj, matOfDMatch.nativeObj, matOfDMatch2.nativeObj, withRotation);
    }

    public static void matchGMS(Size size1, Size size2, MatOfKeyPoint keypoints1, MatOfKeyPoint keypoints2, MatOfDMatch matches1to2, MatOfDMatch matchesGMS) {
        Size size = size1;
        Size size3 = size2;
        MatOfKeyPoint matOfKeyPoint = keypoints1;
        MatOfKeyPoint matOfKeyPoint2 = keypoints2;
        MatOfKeyPoint matOfKeyPoint3 = matOfKeyPoint;
        MatOfKeyPoint matOfKeyPoint4 = matOfKeyPoint2;
        matchGMS_3(size.width, size.height, size3.width, size3.height, matOfKeyPoint.nativeObj, matOfKeyPoint2.nativeObj, matches1to2.nativeObj, matchesGMS.nativeObj);
    }
}
