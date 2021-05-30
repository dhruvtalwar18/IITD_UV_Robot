package org.opencv.features2d;

import java.util.ArrayList;
import java.util.List;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Scalar;
import org.opencv.utils.Converters;

public class Features2d {
    public static final int DrawMatchesFlags_DEFAULT = 0;
    public static final int DrawMatchesFlags_DRAW_OVER_OUTIMG = 1;
    public static final int DrawMatchesFlags_DRAW_RICH_KEYPOINTS = 4;
    public static final int DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS = 2;

    private static native void drawKeypoints_0(long j, long j2, long j3, double d, double d2, double d3, double d4, int i);

    private static native void drawKeypoints_1(long j, long j2, long j3, double d, double d2, double d3, double d4);

    private static native void drawKeypoints_2(long j, long j2, long j3);

    private static native void drawMatchesKnn_0(long j, long j2, long j3, long j4, long j5, long j6, double d, double d2, double d3, double d4, double d5, double d6, double d7, double d8, long j7, int i);

    private static native void drawMatchesKnn_1(long j, long j2, long j3, long j4, long j5, long j6, double d, double d2, double d3, double d4, double d5, double d6, double d7, double d8, long j7);

    private static native void drawMatchesKnn_2(long j, long j2, long j3, long j4, long j5, long j6, double d, double d2, double d3, double d4, double d5, double d6, double d7, double d8);

    private static native void drawMatchesKnn_3(long j, long j2, long j3, long j4, long j5, long j6, double d, double d2, double d3, double d4);

    private static native void drawMatchesKnn_4(long j, long j2, long j3, long j4, long j5, long j6);

    private static native void drawMatches_0(long j, long j2, long j3, long j4, long j5, long j6, double d, double d2, double d3, double d4, double d5, double d6, double d7, double d8, long j7, int i);

    private static native void drawMatches_1(long j, long j2, long j3, long j4, long j5, long j6, double d, double d2, double d3, double d4, double d5, double d6, double d7, double d8, long j7);

    private static native void drawMatches_2(long j, long j2, long j3, long j4, long j5, long j6, double d, double d2, double d3, double d4, double d5, double d6, double d7, double d8);

    private static native void drawMatches_3(long j, long j2, long j3, long j4, long j5, long j6, double d, double d2, double d3, double d4);

    private static native void drawMatches_4(long j, long j2, long j3, long j4, long j5, long j6);

    public static void drawKeypoints(Mat image, MatOfKeyPoint keypoints, Mat outImage, Scalar color, int flags) {
        Scalar scalar = color;
        drawKeypoints_0(image.nativeObj, keypoints.nativeObj, outImage.nativeObj, scalar.val[0], scalar.val[1], scalar.val[2], scalar.val[3], flags);
    }

    public static void drawKeypoints(Mat image, MatOfKeyPoint keypoints, Mat outImage, Scalar color) {
        Scalar scalar = color;
        drawKeypoints_1(image.nativeObj, keypoints.nativeObj, outImage.nativeObj, scalar.val[0], scalar.val[1], scalar.val[2], scalar.val[3]);
    }

    public static void drawKeypoints(Mat image, MatOfKeyPoint keypoints, Mat outImage) {
        drawKeypoints_2(image.nativeObj, keypoints.nativeObj, outImage.nativeObj);
    }

    public static void drawMatches(Mat img1, MatOfKeyPoint keypoints1, Mat img2, MatOfKeyPoint keypoints2, MatOfDMatch matches1to2, Mat outImg, Scalar matchColor, Scalar singlePointColor, MatOfByte matchesMask, int flags) {
        Scalar scalar = matchColor;
        Scalar scalar2 = singlePointColor;
        MatOfKeyPoint matOfKeyPoint = keypoints1;
        MatOfKeyPoint matOfKeyPoint2 = keypoints2;
        MatOfDMatch matOfDMatch = matches1to2;
        long j = img1.nativeObj;
        MatOfDMatch matOfDMatch2 = matOfDMatch;
        MatOfByte matOfByte = matchesMask;
        MatOfKeyPoint matOfKeyPoint3 = matOfKeyPoint;
        MatOfKeyPoint matOfKeyPoint4 = matOfKeyPoint2;
        MatOfByte matOfByte2 = matOfByte;
        long j2 = j;
        drawMatches_0(j2, matOfKeyPoint.nativeObj, img2.nativeObj, matOfKeyPoint2.nativeObj, matOfDMatch.nativeObj, outImg.nativeObj, scalar.val[0], scalar.val[1], scalar.val[2], scalar.val[3], scalar2.val[0], scalar2.val[1], scalar2.val[2], scalar2.val[3], matOfByte.nativeObj, flags);
    }

    public static void drawMatches(Mat img1, MatOfKeyPoint keypoints1, Mat img2, MatOfKeyPoint keypoints2, MatOfDMatch matches1to2, Mat outImg, Scalar matchColor, Scalar singlePointColor, MatOfByte matchesMask) {
        Scalar scalar = matchColor;
        Scalar scalar2 = singlePointColor;
        MatOfKeyPoint matOfKeyPoint = keypoints1;
        MatOfKeyPoint matOfKeyPoint2 = keypoints2;
        MatOfDMatch matOfDMatch = matches1to2;
        MatOfKeyPoint matOfKeyPoint3 = matOfKeyPoint;
        MatOfKeyPoint matOfKeyPoint4 = matOfKeyPoint2;
        MatOfDMatch matOfDMatch2 = matOfDMatch;
        drawMatches_1(img1.nativeObj, matOfKeyPoint.nativeObj, img2.nativeObj, matOfKeyPoint2.nativeObj, matOfDMatch.nativeObj, outImg.nativeObj, scalar.val[0], scalar.val[1], scalar.val[2], scalar.val[3], scalar2.val[0], scalar2.val[1], scalar2.val[2], scalar2.val[3], matchesMask.nativeObj);
    }

    public static void drawMatches(Mat img1, MatOfKeyPoint keypoints1, Mat img2, MatOfKeyPoint keypoints2, MatOfDMatch matches1to2, Mat outImg, Scalar matchColor, Scalar singlePointColor) {
        Scalar scalar = matchColor;
        Scalar scalar2 = singlePointColor;
        MatOfKeyPoint matOfKeyPoint = keypoints1;
        MatOfKeyPoint matOfKeyPoint2 = keypoints2;
        MatOfDMatch matOfDMatch = matches1to2;
        MatOfKeyPoint matOfKeyPoint3 = matOfKeyPoint;
        MatOfKeyPoint matOfKeyPoint4 = matOfKeyPoint2;
        MatOfDMatch matOfDMatch2 = matOfDMatch;
        drawMatches_2(img1.nativeObj, matOfKeyPoint.nativeObj, img2.nativeObj, matOfKeyPoint2.nativeObj, matOfDMatch.nativeObj, outImg.nativeObj, scalar.val[0], scalar.val[1], scalar.val[2], scalar.val[3], scalar2.val[0], scalar2.val[1], scalar2.val[2], scalar2.val[3]);
    }

    public static void drawMatches(Mat img1, MatOfKeyPoint keypoints1, Mat img2, MatOfKeyPoint keypoints2, MatOfDMatch matches1to2, Mat outImg, Scalar matchColor) {
        Scalar scalar = matchColor;
        MatOfKeyPoint matOfKeyPoint = keypoints1;
        MatOfKeyPoint matOfKeyPoint2 = keypoints2;
        MatOfDMatch matOfDMatch = matches1to2;
        MatOfKeyPoint matOfKeyPoint3 = matOfKeyPoint;
        MatOfKeyPoint matOfKeyPoint4 = matOfKeyPoint2;
        MatOfDMatch matOfDMatch2 = matOfDMatch;
        drawMatches_3(img1.nativeObj, matOfKeyPoint.nativeObj, img2.nativeObj, matOfKeyPoint2.nativeObj, matOfDMatch.nativeObj, outImg.nativeObj, scalar.val[0], scalar.val[1], scalar.val[2], scalar.val[3]);
    }

    public static void drawMatches(Mat img1, MatOfKeyPoint keypoints1, Mat img2, MatOfKeyPoint keypoints2, MatOfDMatch matches1to2, Mat outImg) {
        MatOfKeyPoint matOfKeyPoint = keypoints1;
        MatOfKeyPoint matOfKeyPoint2 = keypoints2;
        MatOfKeyPoint matOfKeyPoint3 = matOfKeyPoint;
        MatOfKeyPoint matOfKeyPoint4 = matOfKeyPoint2;
        drawMatches_4(img1.nativeObj, matOfKeyPoint.nativeObj, img2.nativeObj, matOfKeyPoint2.nativeObj, matches1to2.nativeObj, outImg.nativeObj);
    }

    public static void drawMatchesKnn(Mat img1, MatOfKeyPoint keypoints1, Mat img2, MatOfKeyPoint keypoints2, List<MatOfDMatch> matches1to2, Mat outImg, Scalar matchColor, Scalar singlePointColor, List<MatOfByte> matchesMask, int flags) {
        List<MatOfDMatch> list = matches1to2;
        Scalar scalar = matchColor;
        Scalar scalar2 = singlePointColor;
        List<MatOfByte> list2 = matchesMask;
        MatOfKeyPoint matOfKeyPoint = keypoints1;
        MatOfKeyPoint matOfKeyPoint2 = keypoints2;
        MatOfKeyPoint matOfKeyPoint3 = matOfKeyPoint;
        MatOfKeyPoint matOfKeyPoint4 = matOfKeyPoint2;
        drawMatchesKnn_0(img1.nativeObj, matOfKeyPoint.nativeObj, img2.nativeObj, matOfKeyPoint2.nativeObj, Converters.vector_vector_DMatch_to_Mat(list, new ArrayList<>(list != null ? matches1to2.size() : 0)).nativeObj, outImg.nativeObj, scalar.val[0], scalar.val[1], scalar.val[2], scalar.val[3], scalar2.val[0], scalar2.val[1], scalar2.val[2], scalar2.val[3], Converters.vector_vector_char_to_Mat(list2, new ArrayList<>(list2 != null ? matchesMask.size() : 0)).nativeObj, flags);
    }

    public static void drawMatchesKnn(Mat img1, MatOfKeyPoint keypoints1, Mat img2, MatOfKeyPoint keypoints2, List<MatOfDMatch> matches1to2, Mat outImg, Scalar matchColor, Scalar singlePointColor, List<MatOfByte> matchesMask) {
        List<MatOfDMatch> list = matches1to2;
        Scalar scalar = matchColor;
        Scalar scalar2 = singlePointColor;
        List<MatOfByte> list2 = matchesMask;
        MatOfKeyPoint matOfKeyPoint = keypoints1;
        MatOfKeyPoint matOfKeyPoint2 = keypoints2;
        MatOfKeyPoint matOfKeyPoint3 = matOfKeyPoint;
        MatOfKeyPoint matOfKeyPoint4 = matOfKeyPoint2;
        drawMatchesKnn_1(img1.nativeObj, matOfKeyPoint.nativeObj, img2.nativeObj, matOfKeyPoint2.nativeObj, Converters.vector_vector_DMatch_to_Mat(list, new ArrayList<>(list != null ? matches1to2.size() : 0)).nativeObj, outImg.nativeObj, scalar.val[0], scalar.val[1], scalar.val[2], scalar.val[3], scalar2.val[0], scalar2.val[1], scalar2.val[2], scalar2.val[3], Converters.vector_vector_char_to_Mat(list2, new ArrayList<>(list2 != null ? matchesMask.size() : 0)).nativeObj);
    }

    public static void drawMatchesKnn(Mat img1, MatOfKeyPoint keypoints1, Mat img2, MatOfKeyPoint keypoints2, List<MatOfDMatch> matches1to2, Mat outImg, Scalar matchColor, Scalar singlePointColor) {
        List<MatOfDMatch> list = matches1to2;
        Scalar scalar = matchColor;
        Scalar scalar2 = singlePointColor;
        Mat keypoints1_mat = keypoints1;
        Mat keypoints2_mat = keypoints2;
        drawMatchesKnn_2(img1.nativeObj, keypoints1_mat.nativeObj, img2.nativeObj, keypoints2_mat.nativeObj, Converters.vector_vector_DMatch_to_Mat(list, new ArrayList<>(list != null ? matches1to2.size() : 0)).nativeObj, outImg.nativeObj, scalar.val[0], scalar.val[1], scalar.val[2], scalar.val[3], scalar2.val[0], scalar2.val[1], scalar2.val[2], scalar2.val[3]);
    }

    public static void drawMatchesKnn(Mat img1, MatOfKeyPoint keypoints1, Mat img2, MatOfKeyPoint keypoints2, List<MatOfDMatch> matches1to2, Mat outImg, Scalar matchColor) {
        List<MatOfDMatch> list = matches1to2;
        Scalar scalar = matchColor;
        MatOfKeyPoint matOfKeyPoint = keypoints1;
        MatOfKeyPoint matOfKeyPoint2 = keypoints2;
        Mat matches1to2_mat = Converters.vector_vector_DMatch_to_Mat(list, new ArrayList<>(list != null ? matches1to2.size() : 0));
        MatOfKeyPoint matOfKeyPoint3 = matOfKeyPoint;
        MatOfKeyPoint matOfKeyPoint4 = matOfKeyPoint2;
        Mat mat = matches1to2_mat;
        drawMatchesKnn_3(img1.nativeObj, matOfKeyPoint.nativeObj, img2.nativeObj, matOfKeyPoint2.nativeObj, matches1to2_mat.nativeObj, outImg.nativeObj, scalar.val[0], scalar.val[1], scalar.val[2], scalar.val[3]);
    }

    public static void drawMatchesKnn(Mat img1, MatOfKeyPoint keypoints1, Mat img2, MatOfKeyPoint keypoints2, List<MatOfDMatch> matches1to2, Mat outImg) {
        List<MatOfDMatch> list = matches1to2;
        MatOfKeyPoint matOfKeyPoint = keypoints1;
        MatOfKeyPoint matOfKeyPoint2 = keypoints2;
        ArrayList arrayList = new ArrayList(list != null ? matches1to2.size() : 0);
        MatOfKeyPoint matOfKeyPoint3 = matOfKeyPoint;
        MatOfKeyPoint matOfKeyPoint4 = matOfKeyPoint2;
        ArrayList arrayList2 = arrayList;
        drawMatchesKnn_4(img1.nativeObj, matOfKeyPoint.nativeObj, img2.nativeObj, matOfKeyPoint2.nativeObj, Converters.vector_vector_DMatch_to_Mat(list, arrayList).nativeObj, outImg.nativeObj);
    }
}
