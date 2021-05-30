package org.opencv.aruco;

import java.util.List;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.utils.Converters;

public class Aruco {
    public static final int CORNER_REFINE_APRILTAG = 3;
    public static final int CORNER_REFINE_CONTOUR = 2;
    public static final int CORNER_REFINE_NONE = 0;
    public static final int CORNER_REFINE_SUBPIX = 1;
    public static final int DICT_4X4_100 = 1;
    public static final int DICT_4X4_1000 = 3;
    public static final int DICT_4X4_250 = 2;
    public static final int DICT_4X4_50 = 0;
    public static final int DICT_5X5_100 = 5;
    public static final int DICT_5X5_1000 = 7;
    public static final int DICT_5X5_250 = 6;
    public static final int DICT_5X5_50 = 4;
    public static final int DICT_6X6_100 = 9;
    public static final int DICT_6X6_1000 = 11;
    public static final int DICT_6X6_250 = 10;
    public static final int DICT_6X6_50 = 8;
    public static final int DICT_7X7_100 = 13;
    public static final int DICT_7X7_1000 = 15;
    public static final int DICT_7X7_250 = 14;
    public static final int DICT_7X7_50 = 12;
    public static final int DICT_APRILTAG_16h5 = 17;
    public static final int DICT_APRILTAG_25h9 = 18;
    public static final int DICT_APRILTAG_36h10 = 19;
    public static final int DICT_APRILTAG_36h11 = 20;
    public static final int DICT_ARUCO_ORIGINAL = 16;

    private static native double calibrateCameraArucoExtended_0(long j, long j2, long j3, long j4, double d, double d2, long j5, long j6, long j7, long j8, long j9, long j10, long j11, int i, int i2, int i3, double d3);

    private static native double calibrateCameraArucoExtended_1(long j, long j2, long j3, long j4, double d, double d2, long j5, long j6, long j7, long j8, long j9, long j10, long j11, int i);

    private static native double calibrateCameraArucoExtended_2(long j, long j2, long j3, long j4, double d, double d2, long j5, long j6, long j7, long j8, long j9, long j10, long j11);

    private static native double calibrateCameraAruco_0(long j, long j2, long j3, long j4, double d, double d2, long j5, long j6, long j7, long j8, int i, int i2, int i3, double d3);

    private static native double calibrateCameraAruco_1(long j, long j2, long j3, long j4, double d, double d2, long j5, long j6, long j7, long j8, int i);

    private static native double calibrateCameraAruco_2(long j, long j2, long j3, long j4, double d, double d2, long j5, long j6, long j7, long j8);

    private static native double calibrateCameraAruco_3(long j, long j2, long j3, long j4, double d, double d2, long j5, long j6, long j7);

    private static native double calibrateCameraAruco_4(long j, long j2, long j3, long j4, double d, double d2, long j5, long j6);

    private static native double calibrateCameraCharucoExtended_0(long j, long j2, long j3, double d, double d2, long j4, long j5, long j6, long j7, long j8, long j9, long j10, int i, int i2, int i3, double d3);

    private static native double calibrateCameraCharucoExtended_1(long j, long j2, long j3, double d, double d2, long j4, long j5, long j6, long j7, long j8, long j9, long j10, int i);

    private static native double calibrateCameraCharucoExtended_2(long j, long j2, long j3, double d, double d2, long j4, long j5, long j6, long j7, long j8, long j9, long j10);

    private static native double calibrateCameraCharuco_0(long j, long j2, long j3, double d, double d2, long j4, long j5, long j6, long j7, int i, int i2, int i3, double d3);

    private static native double calibrateCameraCharuco_1(long j, long j2, long j3, double d, double d2, long j4, long j5, long j6, long j7, int i);

    private static native double calibrateCameraCharuco_2(long j, long j2, long j3, double d, double d2, long j4, long j5, long j6, long j7);

    private static native double calibrateCameraCharuco_3(long j, long j2, long j3, double d, double d2, long j4, long j5, long j6);

    private static native double calibrateCameraCharuco_4(long j, long j2, long j3, double d, double d2, long j4, long j5);

    private static native long custom_dictionary_0(int i, int i2, int i3);

    private static native long custom_dictionary_1(int i, int i2);

    private static native long custom_dictionary_from_0(int i, int i2, long j, int i3);

    private static native long custom_dictionary_from_1(int i, int i2, long j);

    private static native void detectCharucoDiamond_0(long j, long j2, long j3, float f, long j4, long j5, long j6, long j7);

    private static native void detectCharucoDiamond_1(long j, long j2, long j3, float f, long j4, long j5, long j6);

    private static native void detectCharucoDiamond_2(long j, long j2, long j3, float f, long j4, long j5);

    private static native void detectMarkers_0(long j, long j2, long j3, long j4, long j5, long j6, long j7, long j8);

    private static native void detectMarkers_1(long j, long j2, long j3, long j4, long j5, long j6, long j7);

    private static native void detectMarkers_2(long j, long j2, long j3, long j4, long j5, long j6);

    private static native void detectMarkers_3(long j, long j2, long j3, long j4, long j5);

    private static native void detectMarkers_4(long j, long j2, long j3, long j4);

    private static native void drawAxis_0(long j, long j2, long j3, long j4, long j5, float f);

    private static native void drawDetectedCornersCharuco_0(long j, long j2, long j3, double d, double d2, double d3, double d4);

    private static native void drawDetectedCornersCharuco_1(long j, long j2, long j3);

    private static native void drawDetectedCornersCharuco_2(long j, long j2);

    private static native void drawDetectedDiamonds_0(long j, long j2, long j3, double d, double d2, double d3, double d4);

    private static native void drawDetectedDiamonds_1(long j, long j2, long j3);

    private static native void drawDetectedDiamonds_2(long j, long j2);

    private static native void drawDetectedMarkers_0(long j, long j2, long j3, double d, double d2, double d3, double d4);

    private static native void drawDetectedMarkers_1(long j, long j2, long j3);

    private static native void drawDetectedMarkers_2(long j, long j2);

    private static native void drawMarker_0(long j, int i, int i2, long j2, int i3);

    private static native void drawMarker_1(long j, int i, int i2, long j2);

    private static native void drawPlanarBoard_0(long j, double d, double d2, long j2, int i, int i2);

    private static native void drawPlanarBoard_1(long j, double d, double d2, long j2, int i);

    private static native void drawPlanarBoard_2(long j, double d, double d2, long j2);

    private static native int estimatePoseBoard_0(long j, long j2, long j3, long j4, long j5, long j6, long j7, boolean z);

    private static native int estimatePoseBoard_1(long j, long j2, long j3, long j4, long j5, long j6, long j7);

    private static native boolean estimatePoseCharucoBoard_0(long j, long j2, long j3, long j4, long j5, long j6, long j7, boolean z);

    private static native boolean estimatePoseCharucoBoard_1(long j, long j2, long j3, long j4, long j5, long j6, long j7);

    private static native void estimatePoseSingleMarkers_0(long j, float f, long j2, long j3, long j4, long j5, long j6);

    private static native void estimatePoseSingleMarkers_1(long j, float f, long j2, long j3, long j4, long j5);

    private static native void getBoardObjectAndImagePoints_0(long j, long j2, long j3, long j4, long j5);

    private static native long getPredefinedDictionary_0(int i);

    private static native int interpolateCornersCharuco_0(long j, long j2, long j3, long j4, long j5, long j6, long j7, long j8, int i);

    private static native int interpolateCornersCharuco_1(long j, long j2, long j3, long j4, long j5, long j6, long j7, long j8);

    private static native int interpolateCornersCharuco_2(long j, long j2, long j3, long j4, long j5, long j6, long j7);

    private static native int interpolateCornersCharuco_3(long j, long j2, long j3, long j4, long j5, long j6);

    private static native void refineDetectedMarkers_0(long j, long j2, long j3, long j4, long j5, long j6, long j7, float f, float f2, boolean z, long j8, long j9);

    private static native void refineDetectedMarkers_1(long j, long j2, long j3, long j4, long j5, long j6, long j7, float f, float f2, boolean z, long j8);

    private static native void refineDetectedMarkers_2(long j, long j2, long j3, long j4, long j5, long j6, long j7, float f, float f2, boolean z);

    private static native void refineDetectedMarkers_3(long j, long j2, long j3, long j4, long j5, long j6, long j7, float f, float f2);

    private static native void refineDetectedMarkers_4(long j, long j2, long j3, long j4, long j5, long j6, long j7, float f);

    private static native void refineDetectedMarkers_5(long j, long j2, long j3, long j4, long j5, long j6, long j7);

    private static native void refineDetectedMarkers_6(long j, long j2, long j3, long j4, long j5, long j6);

    private static native void refineDetectedMarkers_7(long j, long j2, long j3, long j4, long j5);

    public static Dictionary custom_dictionary_from(int nMarkers, int markerSize, Dictionary baseDictionary, int randomSeed) {
        return Dictionary.__fromPtr__(custom_dictionary_from_0(nMarkers, markerSize, baseDictionary.getNativeObjAddr(), randomSeed));
    }

    public static Dictionary custom_dictionary_from(int nMarkers, int markerSize, Dictionary baseDictionary) {
        return Dictionary.__fromPtr__(custom_dictionary_from_1(nMarkers, markerSize, baseDictionary.getNativeObjAddr()));
    }

    public static Dictionary custom_dictionary(int nMarkers, int markerSize, int randomSeed) {
        return Dictionary.__fromPtr__(custom_dictionary_0(nMarkers, markerSize, randomSeed));
    }

    public static Dictionary custom_dictionary(int nMarkers, int markerSize) {
        return Dictionary.__fromPtr__(custom_dictionary_1(nMarkers, markerSize));
    }

    public static Dictionary getPredefinedDictionary(int dict) {
        return Dictionary.__fromPtr__(getPredefinedDictionary_0(dict));
    }

    public static boolean estimatePoseCharucoBoard(Mat charucoCorners, Mat charucoIds, CharucoBoard board, Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec, boolean useExtrinsicGuess) {
        return estimatePoseCharucoBoard_0(charucoCorners.nativeObj, charucoIds.nativeObj, board.getNativeObjAddr(), cameraMatrix.nativeObj, distCoeffs.nativeObj, rvec.nativeObj, tvec.nativeObj, useExtrinsicGuess);
    }

    public static boolean estimatePoseCharucoBoard(Mat charucoCorners, Mat charucoIds, CharucoBoard board, Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec) {
        return estimatePoseCharucoBoard_1(charucoCorners.nativeObj, charucoIds.nativeObj, board.getNativeObjAddr(), cameraMatrix.nativeObj, distCoeffs.nativeObj, rvec.nativeObj, tvec.nativeObj);
    }

    public static double calibrateCameraArucoExtended(List<Mat> corners, Mat ids, Mat counter, Board board, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, Mat stdDeviationsIntrinsics, Mat stdDeviationsExtrinsics, Mat perViewErrors, int flags, TermCriteria criteria) {
        Size size = imageSize;
        TermCriteria termCriteria = criteria;
        Mat corners_mat = Converters.vector_Mat_to_Mat(corners);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        long j = corners_mat.nativeObj;
        long j2 = ids.nativeObj;
        long j3 = counter.nativeObj;
        long nativeObjAddr = board.getNativeObjAddr();
        double d = size.width;
        Mat tvecs_mat2 = tvecs_mat;
        double d2 = size.height;
        Mat tvecs_mat3 = tvecs_mat2;
        Mat corners_mat2 = corners_mat;
        long j4 = cameraMatrix.nativeObj;
        Mat mat = corners_mat2;
        Mat rvecs_mat2 = rvecs_mat;
        long j5 = distCoeffs.nativeObj;
        long j6 = rvecs_mat2.nativeObj;
        long j7 = tvecs_mat3.nativeObj;
        long j8 = stdDeviationsIntrinsics.nativeObj;
        long j9 = stdDeviationsExtrinsics.nativeObj;
        long j10 = perViewErrors.nativeObj;
        int i = termCriteria.type;
        int i2 = termCriteria.maxCount;
        double d3 = termCriteria.epsilon;
        Mat rvecs_mat3 = rvecs_mat2;
        double retVal = calibrateCameraArucoExtended_0(j, j2, j3, nativeObjAddr, d, d2, j4, j5, j6, j7, j8, j9, j10, flags, i, i2, d3);
        Converters.Mat_to_vector_Mat(rvecs_mat3, rvecs);
        rvecs_mat3.release();
        Converters.Mat_to_vector_Mat(tvecs_mat3, tvecs);
        tvecs_mat3.release();
        return retVal;
    }

    public static double calibrateCameraArucoExtended(List<Mat> corners, Mat ids, Mat counter, Board board, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, Mat stdDeviationsIntrinsics, Mat stdDeviationsExtrinsics, Mat perViewErrors, int flags) {
        Size size = imageSize;
        Mat corners_mat = Converters.vector_Mat_to_Mat(corners);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        long j = corners_mat.nativeObj;
        long j2 = ids.nativeObj;
        Mat rvecs_mat2 = rvecs_mat;
        long j3 = j;
        Mat rvecs_mat3 = rvecs_mat2;
        Mat tvecs_mat2 = tvecs_mat;
        Mat mat = corners_mat;
        Mat tvecs_mat3 = tvecs_mat2;
        long j4 = j3;
        long j5 = j2;
        double retVal = calibrateCameraArucoExtended_1(j4, j5, counter.nativeObj, board.getNativeObjAddr(), size.width, size.height, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat3.nativeObj, tvecs_mat2.nativeObj, stdDeviationsIntrinsics.nativeObj, stdDeviationsExtrinsics.nativeObj, perViewErrors.nativeObj, flags);
        Converters.Mat_to_vector_Mat(rvecs_mat3, rvecs);
        rvecs_mat3.release();
        Mat tvecs_mat4 = tvecs_mat3;
        Converters.Mat_to_vector_Mat(tvecs_mat4, tvecs);
        tvecs_mat4.release();
        return retVal;
    }

    public static double calibrateCameraArucoExtended(List<Mat> corners, Mat ids, Mat counter, Board board, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, Mat stdDeviationsIntrinsics, Mat stdDeviationsExtrinsics, Mat perViewErrors) {
        Size size = imageSize;
        Mat corners_mat = Converters.vector_Mat_to_Mat(corners);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        Mat mat = corners_mat;
        double retVal = calibrateCameraArucoExtended_2(corners_mat.nativeObj, ids.nativeObj, counter.nativeObj, board.getNativeObjAddr(), size.width, size.height, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, stdDeviationsIntrinsics.nativeObj, stdDeviationsExtrinsics.nativeObj, perViewErrors.nativeObj);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    public static double calibrateCameraAruco(List<Mat> corners, Mat ids, Mat counter, Board board, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, int flags, TermCriteria criteria) {
        Size size = imageSize;
        TermCriteria termCriteria = criteria;
        Mat corners_mat = Converters.vector_Mat_to_Mat(corners);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        long j = corners_mat.nativeObj;
        long j2 = ids.nativeObj;
        long j3 = counter.nativeObj;
        long nativeObjAddr = board.getNativeObjAddr();
        double d = size.width;
        Mat tvecs_mat2 = tvecs_mat;
        double d2 = size.height;
        Mat tvecs_mat3 = tvecs_mat2;
        Mat corners_mat2 = corners_mat;
        long j4 = cameraMatrix.nativeObj;
        Mat mat = corners_mat2;
        Mat rvecs_mat2 = rvecs_mat;
        long j5 = distCoeffs.nativeObj;
        long j6 = rvecs_mat2.nativeObj;
        long j7 = tvecs_mat3.nativeObj;
        int i = termCriteria.type;
        int i2 = termCriteria.maxCount;
        double d3 = termCriteria.epsilon;
        Mat rvecs_mat3 = rvecs_mat2;
        double retVal = calibrateCameraAruco_0(j, j2, j3, nativeObjAddr, d, d2, j4, j5, j6, j7, flags, i, i2, d3);
        Converters.Mat_to_vector_Mat(rvecs_mat3, rvecs);
        rvecs_mat3.release();
        Converters.Mat_to_vector_Mat(tvecs_mat3, tvecs);
        tvecs_mat3.release();
        return retVal;
    }

    public static double calibrateCameraAruco(List<Mat> corners, Mat ids, Mat counter, Board board, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, int flags) {
        Size size = imageSize;
        Mat corners_mat = Converters.vector_Mat_to_Mat(corners);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        long j = corners_mat.nativeObj;
        long j2 = ids.nativeObj;
        long j3 = counter.nativeObj;
        long nativeObjAddr = board.getNativeObjAddr();
        double d = size.width;
        double d2 = size.height;
        Mat rvecs_mat2 = rvecs_mat;
        long j4 = cameraMatrix.nativeObj;
        long j5 = j;
        Mat rvecs_mat3 = rvecs_mat2;
        Mat tvecs_mat2 = tvecs_mat;
        Mat mat = corners_mat;
        long j6 = distCoeffs.nativeObj;
        long j7 = rvecs_mat3.nativeObj;
        long j8 = tvecs_mat2.nativeObj;
        Mat tvecs_mat3 = tvecs_mat2;
        double retVal = calibrateCameraAruco_1(j5, j2, j3, nativeObjAddr, d, d2, j4, j6, j7, j8, flags);
        Converters.Mat_to_vector_Mat(rvecs_mat3, rvecs);
        rvecs_mat3.release();
        Mat tvecs_mat4 = tvecs_mat3;
        Converters.Mat_to_vector_Mat(tvecs_mat4, tvecs);
        tvecs_mat4.release();
        return retVal;
    }

    public static double calibrateCameraAruco(List<Mat> corners, Mat ids, Mat counter, Board board, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs) {
        Size size = imageSize;
        Mat corners_mat = Converters.vector_Mat_to_Mat(corners);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        Mat mat = corners_mat;
        double retVal = calibrateCameraAruco_2(corners_mat.nativeObj, ids.nativeObj, counter.nativeObj, board.getNativeObjAddr(), size.width, size.height, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    public static double calibrateCameraAruco(List<Mat> corners, Mat ids, Mat counter, Board board, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs) {
        Size size = imageSize;
        Mat corners_mat = Converters.vector_Mat_to_Mat(corners);
        Mat rvecs_mat = new Mat();
        Mat mat = corners_mat;
        double retVal = calibrateCameraAruco_3(corners_mat.nativeObj, ids.nativeObj, counter.nativeObj, board.getNativeObjAddr(), size.width, size.height, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        return retVal;
    }

    public static double calibrateCameraAruco(List<Mat> corners, Mat ids, Mat counter, Board board, Size imageSize, Mat cameraMatrix, Mat distCoeffs) {
        Size size = imageSize;
        return calibrateCameraAruco_4(Converters.vector_Mat_to_Mat(corners).nativeObj, ids.nativeObj, counter.nativeObj, board.getNativeObjAddr(), size.width, size.height, cameraMatrix.nativeObj, distCoeffs.nativeObj);
    }

    public static double calibrateCameraCharucoExtended(List<Mat> charucoCorners, List<Mat> charucoIds, CharucoBoard board, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, Mat stdDeviationsIntrinsics, Mat stdDeviationsExtrinsics, Mat perViewErrors, int flags, TermCriteria criteria) {
        Size size = imageSize;
        TermCriteria termCriteria = criteria;
        Mat charucoCorners_mat = Converters.vector_Mat_to_Mat(charucoCorners);
        Mat charucoIds_mat = Converters.vector_Mat_to_Mat(charucoIds);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        long j = charucoCorners_mat.nativeObj;
        long j2 = charucoIds_mat.nativeObj;
        Mat rvecs_mat2 = rvecs_mat;
        long j3 = j;
        Mat rvecs_mat3 = rvecs_mat2;
        Mat tvecs_mat2 = tvecs_mat;
        Mat charucoCorners_mat2 = charucoCorners_mat;
        Mat mat = charucoCorners_mat2;
        Mat mat2 = charucoIds_mat;
        Mat tvecs_mat3 = tvecs_mat2;
        long j4 = j3;
        long j5 = j2;
        Mat tvecs_mat4 = tvecs_mat3;
        double retVal = calibrateCameraCharucoExtended_0(j4, j5, board.getNativeObjAddr(), size.width, size.height, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat3.nativeObj, tvecs_mat2.nativeObj, stdDeviationsIntrinsics.nativeObj, stdDeviationsExtrinsics.nativeObj, perViewErrors.nativeObj, flags, termCriteria.type, termCriteria.maxCount, termCriteria.epsilon);
        Converters.Mat_to_vector_Mat(rvecs_mat3, rvecs);
        rvecs_mat3.release();
        Converters.Mat_to_vector_Mat(tvecs_mat4, tvecs);
        tvecs_mat4.release();
        return retVal;
    }

    public static double calibrateCameraCharucoExtended(List<Mat> charucoCorners, List<Mat> charucoIds, CharucoBoard board, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, Mat stdDeviationsIntrinsics, Mat stdDeviationsExtrinsics, Mat perViewErrors, int flags) {
        Size size = imageSize;
        Mat charucoCorners_mat = Converters.vector_Mat_to_Mat(charucoCorners);
        Mat charucoIds_mat = Converters.vector_Mat_to_Mat(charucoIds);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        long j = charucoCorners_mat.nativeObj;
        long j2 = charucoIds_mat.nativeObj;
        long nativeObjAddr = board.getNativeObjAddr();
        double d = size.width;
        double d2 = size.height;
        Mat mat = charucoIds_mat;
        Mat tvecs_mat2 = tvecs_mat;
        long j3 = j;
        Mat rvecs_mat2 = rvecs_mat;
        Mat mat2 = charucoCorners_mat;
        Mat rvecs_mat3 = rvecs_mat2;
        Mat tvecs_mat3 = tvecs_mat2;
        long j4 = j3;
        long j5 = j2;
        Mat tvecs_mat4 = rvecs_mat3;
        double retVal = calibrateCameraCharucoExtended_1(j4, j5, nativeObjAddr, d, d2, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat2.nativeObj, tvecs_mat2.nativeObj, stdDeviationsIntrinsics.nativeObj, stdDeviationsExtrinsics.nativeObj, perViewErrors.nativeObj, flags);
        Converters.Mat_to_vector_Mat(tvecs_mat4, rvecs);
        tvecs_mat4.release();
        Mat tvecs_mat5 = tvecs_mat3;
        Converters.Mat_to_vector_Mat(tvecs_mat5, tvecs);
        tvecs_mat5.release();
        return retVal;
    }

    public static double calibrateCameraCharucoExtended(List<Mat> charucoCorners, List<Mat> charucoIds, CharucoBoard board, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, Mat stdDeviationsIntrinsics, Mat stdDeviationsExtrinsics, Mat perViewErrors) {
        Size size = imageSize;
        Mat charucoCorners_mat = Converters.vector_Mat_to_Mat(charucoCorners);
        Mat charucoIds_mat = Converters.vector_Mat_to_Mat(charucoIds);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        Mat mat = charucoCorners_mat;
        Mat mat2 = charucoIds_mat;
        double retVal = calibrateCameraCharucoExtended_2(charucoCorners_mat.nativeObj, charucoIds_mat.nativeObj, board.getNativeObjAddr(), size.width, size.height, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, stdDeviationsIntrinsics.nativeObj, stdDeviationsExtrinsics.nativeObj, perViewErrors.nativeObj);
        Mat rvecs_mat2 = rvecs_mat;
        Converters.Mat_to_vector_Mat(rvecs_mat2, rvecs);
        rvecs_mat2.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    public static double calibrateCameraCharuco(List<Mat> charucoCorners, List<Mat> charucoIds, CharucoBoard board, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, int flags, TermCriteria criteria) {
        Size size = imageSize;
        TermCriteria termCriteria = criteria;
        Mat charucoCorners_mat = Converters.vector_Mat_to_Mat(charucoCorners);
        Mat charucoIds_mat = Converters.vector_Mat_to_Mat(charucoIds);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        long j = charucoCorners_mat.nativeObj;
        long j2 = charucoIds_mat.nativeObj;
        long nativeObjAddr = board.getNativeObjAddr();
        double d = size.width;
        double d2 = size.height;
        Mat rvecs_mat2 = rvecs_mat;
        long j3 = cameraMatrix.nativeObj;
        long j4 = j;
        Mat rvecs_mat3 = rvecs_mat2;
        Mat tvecs_mat2 = tvecs_mat;
        Mat charucoCorners_mat2 = charucoCorners_mat;
        long j5 = distCoeffs.nativeObj;
        Mat mat = charucoCorners_mat2;
        Mat mat2 = charucoIds_mat;
        long j6 = rvecs_mat3.nativeObj;
        long j7 = tvecs_mat2.nativeObj;
        Mat tvecs_mat3 = tvecs_mat2;
        double retVal = calibrateCameraCharuco_0(j4, j2, nativeObjAddr, d, d2, j3, j5, j6, j7, flags, termCriteria.type, termCriteria.maxCount, termCriteria.epsilon);
        Converters.Mat_to_vector_Mat(rvecs_mat3, rvecs);
        rvecs_mat3.release();
        Converters.Mat_to_vector_Mat(tvecs_mat3, tvecs);
        tvecs_mat3.release();
        return retVal;
    }

    public static double calibrateCameraCharuco(List<Mat> charucoCorners, List<Mat> charucoIds, CharucoBoard board, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, int flags) {
        Size size = imageSize;
        Mat charucoCorners_mat = Converters.vector_Mat_to_Mat(charucoCorners);
        Mat charucoIds_mat = Converters.vector_Mat_to_Mat(charucoIds);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        long j = charucoCorners_mat.nativeObj;
        long j2 = charucoIds_mat.nativeObj;
        long nativeObjAddr = board.getNativeObjAddr();
        double d = size.width;
        double d2 = size.height;
        Mat mat = charucoIds_mat;
        Mat tvecs_mat2 = tvecs_mat;
        long j3 = cameraMatrix.nativeObj;
        long j4 = j;
        Mat rvecs_mat2 = rvecs_mat;
        long j5 = distCoeffs.nativeObj;
        Mat mat2 = charucoCorners_mat;
        long j6 = rvecs_mat2.nativeObj;
        long j7 = tvecs_mat2.nativeObj;
        Mat tvecs_mat3 = tvecs_mat2;
        Mat tvecs_mat4 = rvecs_mat2;
        double retVal = calibrateCameraCharuco_1(j4, j2, nativeObjAddr, d, d2, j3, j5, j6, j7, flags);
        Converters.Mat_to_vector_Mat(tvecs_mat4, rvecs);
        tvecs_mat4.release();
        Mat tvecs_mat5 = tvecs_mat3;
        Converters.Mat_to_vector_Mat(tvecs_mat5, tvecs);
        tvecs_mat5.release();
        return retVal;
    }

    public static double calibrateCameraCharuco(List<Mat> charucoCorners, List<Mat> charucoIds, CharucoBoard board, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs) {
        Size size = imageSize;
        Mat charucoCorners_mat = Converters.vector_Mat_to_Mat(charucoCorners);
        Mat charucoIds_mat = Converters.vector_Mat_to_Mat(charucoIds);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        Mat mat = charucoCorners_mat;
        Mat mat2 = charucoIds_mat;
        double retVal = calibrateCameraCharuco_2(charucoCorners_mat.nativeObj, charucoIds_mat.nativeObj, board.getNativeObjAddr(), size.width, size.height, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    public static double calibrateCameraCharuco(List<Mat> charucoCorners, List<Mat> charucoIds, CharucoBoard board, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs) {
        Size size = imageSize;
        Mat charucoCorners_mat = Converters.vector_Mat_to_Mat(charucoCorners);
        Mat charucoIds_mat = Converters.vector_Mat_to_Mat(charucoIds);
        Mat rvecs_mat = new Mat();
        Mat mat = charucoCorners_mat;
        double retVal = calibrateCameraCharuco_3(charucoCorners_mat.nativeObj, charucoIds_mat.nativeObj, board.getNativeObjAddr(), size.width, size.height, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        return retVal;
    }

    public static double calibrateCameraCharuco(List<Mat> charucoCorners, List<Mat> charucoIds, CharucoBoard board, Size imageSize, Mat cameraMatrix, Mat distCoeffs) {
        Size size = imageSize;
        Mat charucoCorners_mat = Converters.vector_Mat_to_Mat(charucoCorners);
        Mat charucoIds_mat = Converters.vector_Mat_to_Mat(charucoIds);
        Mat mat = charucoCorners_mat;
        Mat mat2 = charucoIds_mat;
        return calibrateCameraCharuco_4(charucoCorners_mat.nativeObj, charucoIds_mat.nativeObj, board.getNativeObjAddr(), size.width, size.height, cameraMatrix.nativeObj, distCoeffs.nativeObj);
    }

    public static int estimatePoseBoard(List<Mat> corners, Mat ids, Board board, Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec, boolean useExtrinsicGuess) {
        Mat corners_mat = Converters.vector_Mat_to_Mat(corners);
        Mat mat = corners_mat;
        return estimatePoseBoard_0(corners_mat.nativeObj, ids.nativeObj, board.getNativeObjAddr(), cameraMatrix.nativeObj, distCoeffs.nativeObj, rvec.nativeObj, tvec.nativeObj, useExtrinsicGuess);
    }

    public static int estimatePoseBoard(List<Mat> corners, Mat ids, Board board, Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec) {
        Mat corners_mat = Converters.vector_Mat_to_Mat(corners);
        Mat mat = corners_mat;
        return estimatePoseBoard_1(corners_mat.nativeObj, ids.nativeObj, board.getNativeObjAddr(), cameraMatrix.nativeObj, distCoeffs.nativeObj, rvec.nativeObj, tvec.nativeObj);
    }

    public static int interpolateCornersCharuco(List<Mat> markerCorners, Mat markerIds, Mat image, CharucoBoard board, Mat charucoCorners, Mat charucoIds, Mat cameraMatrix, Mat distCoeffs, int minMarkers) {
        Mat markerCorners_mat = Converters.vector_Mat_to_Mat(markerCorners);
        long j = markerCorners_mat.nativeObj;
        Mat mat = markerCorners_mat;
        return interpolateCornersCharuco_0(j, markerIds.nativeObj, image.nativeObj, board.getNativeObjAddr(), charucoCorners.nativeObj, charucoIds.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, minMarkers);
    }

    public static int interpolateCornersCharuco(List<Mat> markerCorners, Mat markerIds, Mat image, CharucoBoard board, Mat charucoCorners, Mat charucoIds, Mat cameraMatrix, Mat distCoeffs) {
        Mat markerCorners_mat = Converters.vector_Mat_to_Mat(markerCorners);
        Mat mat = markerCorners_mat;
        return interpolateCornersCharuco_1(markerCorners_mat.nativeObj, markerIds.nativeObj, image.nativeObj, board.getNativeObjAddr(), charucoCorners.nativeObj, charucoIds.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj);
    }

    public static int interpolateCornersCharuco(List<Mat> markerCorners, Mat markerIds, Mat image, CharucoBoard board, Mat charucoCorners, Mat charucoIds, Mat cameraMatrix) {
        Mat markerCorners_mat = Converters.vector_Mat_to_Mat(markerCorners);
        Mat mat = markerCorners_mat;
        return interpolateCornersCharuco_2(markerCorners_mat.nativeObj, markerIds.nativeObj, image.nativeObj, board.getNativeObjAddr(), charucoCorners.nativeObj, charucoIds.nativeObj, cameraMatrix.nativeObj);
    }

    public static int interpolateCornersCharuco(List<Mat> markerCorners, Mat markerIds, Mat image, CharucoBoard board, Mat charucoCorners, Mat charucoIds) {
        return interpolateCornersCharuco_3(Converters.vector_Mat_to_Mat(markerCorners).nativeObj, markerIds.nativeObj, image.nativeObj, board.getNativeObjAddr(), charucoCorners.nativeObj, charucoIds.nativeObj);
    }

    public static void detectCharucoDiamond(Mat image, List<Mat> markerCorners, Mat markerIds, float squareMarkerLengthRate, List<Mat> diamondCorners, Mat diamondIds, Mat cameraMatrix, Mat distCoeffs) {
        Mat markerCorners_mat = Converters.vector_Mat_to_Mat(markerCorners);
        Mat diamondCorners_mat = new Mat();
        Mat mat = markerCorners_mat;
        detectCharucoDiamond_0(image.nativeObj, markerCorners_mat.nativeObj, markerIds.nativeObj, squareMarkerLengthRate, diamondCorners_mat.nativeObj, diamondIds.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj);
        Mat diamondCorners_mat2 = diamondCorners_mat;
        Converters.Mat_to_vector_Mat(diamondCorners_mat2, diamondCorners);
        diamondCorners_mat2.release();
    }

    public static void detectCharucoDiamond(Mat image, List<Mat> markerCorners, Mat markerIds, float squareMarkerLengthRate, List<Mat> diamondCorners, Mat diamondIds, Mat cameraMatrix) {
        Mat markerCorners_mat = Converters.vector_Mat_to_Mat(markerCorners);
        Mat diamondCorners_mat = new Mat();
        detectCharucoDiamond_1(image.nativeObj, markerCorners_mat.nativeObj, markerIds.nativeObj, squareMarkerLengthRate, diamondCorners_mat.nativeObj, diamondIds.nativeObj, cameraMatrix.nativeObj);
        Converters.Mat_to_vector_Mat(diamondCorners_mat, diamondCorners);
        diamondCorners_mat.release();
    }

    public static void detectCharucoDiamond(Mat image, List<Mat> markerCorners, Mat markerIds, float squareMarkerLengthRate, List<Mat> diamondCorners, Mat diamondIds) {
        Mat markerCorners_mat = Converters.vector_Mat_to_Mat(markerCorners);
        Mat diamondCorners_mat = new Mat();
        detectCharucoDiamond_2(image.nativeObj, markerCorners_mat.nativeObj, markerIds.nativeObj, squareMarkerLengthRate, diamondCorners_mat.nativeObj, diamondIds.nativeObj);
        Converters.Mat_to_vector_Mat(diamondCorners_mat, diamondCorners);
        diamondCorners_mat.release();
    }

    public static void detectMarkers(Mat image, Dictionary dictionary, List<Mat> corners, Mat ids, DetectorParameters parameters, List<Mat> rejectedImgPoints, Mat cameraMatrix, Mat distCoeff) {
        Mat corners_mat = new Mat();
        Mat rejectedImgPoints_mat = new Mat();
        Mat corners_mat2 = corners_mat;
        Mat rejectedImgPoints_mat2 = rejectedImgPoints_mat;
        detectMarkers_0(image.nativeObj, dictionary.getNativeObjAddr(), corners_mat.nativeObj, ids.nativeObj, parameters.getNativeObjAddr(), rejectedImgPoints_mat.nativeObj, cameraMatrix.nativeObj, distCoeff.nativeObj);
        Mat corners_mat3 = corners_mat2;
        Converters.Mat_to_vector_Mat(corners_mat3, corners);
        corners_mat3.release();
        Mat rejectedImgPoints_mat3 = rejectedImgPoints_mat2;
        Converters.Mat_to_vector_Mat(rejectedImgPoints_mat3, rejectedImgPoints);
        rejectedImgPoints_mat3.release();
    }

    public static void detectMarkers(Mat image, Dictionary dictionary, List<Mat> corners, Mat ids, DetectorParameters parameters, List<Mat> rejectedImgPoints, Mat cameraMatrix) {
        Mat corners_mat = new Mat();
        Mat rejectedImgPoints_mat = new Mat();
        Mat rejectedImgPoints_mat2 = rejectedImgPoints_mat;
        detectMarkers_1(image.nativeObj, dictionary.getNativeObjAddr(), corners_mat.nativeObj, ids.nativeObj, parameters.getNativeObjAddr(), rejectedImgPoints_mat.nativeObj, cameraMatrix.nativeObj);
        Mat corners_mat2 = corners_mat;
        Converters.Mat_to_vector_Mat(corners_mat2, corners);
        corners_mat2.release();
        Mat rejectedImgPoints_mat3 = rejectedImgPoints_mat2;
        Converters.Mat_to_vector_Mat(rejectedImgPoints_mat3, rejectedImgPoints);
        rejectedImgPoints_mat3.release();
    }

    public static void detectMarkers(Mat image, Dictionary dictionary, List<Mat> corners, Mat ids, DetectorParameters parameters, List<Mat> rejectedImgPoints) {
        Mat corners_mat = new Mat();
        Mat rejectedImgPoints_mat = new Mat();
        detectMarkers_2(image.nativeObj, dictionary.getNativeObjAddr(), corners_mat.nativeObj, ids.nativeObj, parameters.getNativeObjAddr(), rejectedImgPoints_mat.nativeObj);
        Converters.Mat_to_vector_Mat(corners_mat, corners);
        corners_mat.release();
        Converters.Mat_to_vector_Mat(rejectedImgPoints_mat, rejectedImgPoints);
        rejectedImgPoints_mat.release();
    }

    public static void detectMarkers(Mat image, Dictionary dictionary, List<Mat> corners, Mat ids, DetectorParameters parameters) {
        Mat corners_mat = new Mat();
        detectMarkers_3(image.nativeObj, dictionary.getNativeObjAddr(), corners_mat.nativeObj, ids.nativeObj, parameters.getNativeObjAddr());
        Converters.Mat_to_vector_Mat(corners_mat, corners);
        corners_mat.release();
    }

    public static void detectMarkers(Mat image, Dictionary dictionary, List<Mat> corners, Mat ids) {
        Mat corners_mat = new Mat();
        detectMarkers_4(image.nativeObj, dictionary.getNativeObjAddr(), corners_mat.nativeObj, ids.nativeObj);
        Converters.Mat_to_vector_Mat(corners_mat, corners);
        corners_mat.release();
    }

    public static void drawAxis(Mat image, Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec, float length) {
        drawAxis_0(image.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvec.nativeObj, tvec.nativeObj, length);
    }

    public static void drawDetectedCornersCharuco(Mat image, Mat charucoCorners, Mat charucoIds, Scalar cornerColor) {
        Scalar scalar = cornerColor;
        drawDetectedCornersCharuco_0(image.nativeObj, charucoCorners.nativeObj, charucoIds.nativeObj, scalar.val[0], scalar.val[1], scalar.val[2], scalar.val[3]);
    }

    public static void drawDetectedCornersCharuco(Mat image, Mat charucoCorners, Mat charucoIds) {
        drawDetectedCornersCharuco_1(image.nativeObj, charucoCorners.nativeObj, charucoIds.nativeObj);
    }

    public static void drawDetectedCornersCharuco(Mat image, Mat charucoCorners) {
        drawDetectedCornersCharuco_2(image.nativeObj, charucoCorners.nativeObj);
    }

    public static void drawDetectedDiamonds(Mat image, List<Mat> diamondCorners, Mat diamondIds, Scalar borderColor) {
        Scalar scalar = borderColor;
        Mat diamondCorners_mat = Converters.vector_Mat_to_Mat(diamondCorners);
        drawDetectedDiamonds_0(image.nativeObj, diamondCorners_mat.nativeObj, diamondIds.nativeObj, scalar.val[0], scalar.val[1], scalar.val[2], scalar.val[3]);
    }

    public static void drawDetectedDiamonds(Mat image, List<Mat> diamondCorners, Mat diamondIds) {
        drawDetectedDiamonds_1(image.nativeObj, Converters.vector_Mat_to_Mat(diamondCorners).nativeObj, diamondIds.nativeObj);
    }

    public static void drawDetectedDiamonds(Mat image, List<Mat> diamondCorners) {
        drawDetectedDiamonds_2(image.nativeObj, Converters.vector_Mat_to_Mat(diamondCorners).nativeObj);
    }

    public static void drawDetectedMarkers(Mat image, List<Mat> corners, Mat ids, Scalar borderColor) {
        Scalar scalar = borderColor;
        Mat corners_mat = Converters.vector_Mat_to_Mat(corners);
        drawDetectedMarkers_0(image.nativeObj, corners_mat.nativeObj, ids.nativeObj, scalar.val[0], scalar.val[1], scalar.val[2], scalar.val[3]);
    }

    public static void drawDetectedMarkers(Mat image, List<Mat> corners, Mat ids) {
        drawDetectedMarkers_1(image.nativeObj, Converters.vector_Mat_to_Mat(corners).nativeObj, ids.nativeObj);
    }

    public static void drawDetectedMarkers(Mat image, List<Mat> corners) {
        drawDetectedMarkers_2(image.nativeObj, Converters.vector_Mat_to_Mat(corners).nativeObj);
    }

    public static void drawMarker(Dictionary dictionary, int id, int sidePixels, Mat img, int borderBits) {
        drawMarker_0(dictionary.getNativeObjAddr(), id, sidePixels, img.nativeObj, borderBits);
    }

    public static void drawMarker(Dictionary dictionary, int id, int sidePixels, Mat img) {
        drawMarker_1(dictionary.getNativeObjAddr(), id, sidePixels, img.nativeObj);
    }

    public static void drawPlanarBoard(Board board, Size outSize, Mat img, int marginSize, int borderBits) {
        drawPlanarBoard_0(board.getNativeObjAddr(), outSize.width, outSize.height, img.nativeObj, marginSize, borderBits);
    }

    public static void drawPlanarBoard(Board board, Size outSize, Mat img, int marginSize) {
        drawPlanarBoard_1(board.getNativeObjAddr(), outSize.width, outSize.height, img.nativeObj, marginSize);
    }

    public static void drawPlanarBoard(Board board, Size outSize, Mat img) {
        drawPlanarBoard_2(board.getNativeObjAddr(), outSize.width, outSize.height, img.nativeObj);
    }

    public static void estimatePoseSingleMarkers(List<Mat> corners, float markerLength, Mat cameraMatrix, Mat distCoeffs, Mat rvecs, Mat tvecs, Mat _objPoints) {
        estimatePoseSingleMarkers_0(Converters.vector_Mat_to_Mat(corners).nativeObj, markerLength, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs.nativeObj, tvecs.nativeObj, _objPoints.nativeObj);
    }

    public static void estimatePoseSingleMarkers(List<Mat> corners, float markerLength, Mat cameraMatrix, Mat distCoeffs, Mat rvecs, Mat tvecs) {
        estimatePoseSingleMarkers_1(Converters.vector_Mat_to_Mat(corners).nativeObj, markerLength, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs.nativeObj, tvecs.nativeObj);
    }

    public static void getBoardObjectAndImagePoints(Board board, List<Mat> detectedCorners, Mat detectedIds, Mat objPoints, Mat imgPoints) {
        getBoardObjectAndImagePoints_0(board.getNativeObjAddr(), Converters.vector_Mat_to_Mat(detectedCorners).nativeObj, detectedIds.nativeObj, objPoints.nativeObj, imgPoints.nativeObj);
    }

    public static void refineDetectedMarkers(Mat image, Board board, List<Mat> detectedCorners, Mat detectedIds, List<Mat> rejectedCorners, Mat cameraMatrix, Mat distCoeffs, float minRepDistance, float errorCorrectionRate, boolean checkAllOrders, Mat recoveredIdxs, DetectorParameters parameters) {
        Mat detectedCorners_mat = Converters.vector_Mat_to_Mat(detectedCorners);
        Mat rejectedCorners_mat = Converters.vector_Mat_to_Mat(rejectedCorners);
        long j = image.nativeObj;
        long nativeObjAddr = board.getNativeObjAddr();
        Mat detectedCorners_mat2 = detectedCorners_mat;
        long j2 = j;
        long j3 = nativeObjAddr;
        refineDetectedMarkers_0(j2, j3, detectedCorners_mat.nativeObj, detectedIds.nativeObj, rejectedCorners_mat.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, minRepDistance, errorCorrectionRate, checkAllOrders, recoveredIdxs.nativeObj, parameters.getNativeObjAddr());
        Mat detectedCorners_mat3 = detectedCorners_mat2;
        Converters.Mat_to_vector_Mat(detectedCorners_mat3, detectedCorners);
        detectedCorners_mat3.release();
        Mat rejectedCorners_mat2 = rejectedCorners_mat;
        Converters.Mat_to_vector_Mat(rejectedCorners_mat2, rejectedCorners);
        rejectedCorners_mat2.release();
    }

    public static void refineDetectedMarkers(Mat image, Board board, List<Mat> detectedCorners, Mat detectedIds, List<Mat> rejectedCorners, Mat cameraMatrix, Mat distCoeffs, float minRepDistance, float errorCorrectionRate, boolean checkAllOrders, Mat recoveredIdxs) {
        Mat detectedCorners_mat = Converters.vector_Mat_to_Mat(detectedCorners);
        Mat rejectedCorners_mat = Converters.vector_Mat_to_Mat(rejectedCorners);
        long j = image.nativeObj;
        long nativeObjAddr = board.getNativeObjAddr();
        Mat detectedCorners_mat2 = detectedCorners_mat;
        long j2 = j;
        long j3 = nativeObjAddr;
        refineDetectedMarkers_1(j2, j3, detectedCorners_mat.nativeObj, detectedIds.nativeObj, rejectedCorners_mat.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, minRepDistance, errorCorrectionRate, checkAllOrders, recoveredIdxs.nativeObj);
        Mat detectedCorners_mat3 = detectedCorners_mat2;
        Converters.Mat_to_vector_Mat(detectedCorners_mat3, detectedCorners);
        detectedCorners_mat3.release();
        Mat rejectedCorners_mat2 = rejectedCorners_mat;
        Converters.Mat_to_vector_Mat(rejectedCorners_mat2, rejectedCorners);
        rejectedCorners_mat2.release();
    }

    public static void refineDetectedMarkers(Mat image, Board board, List<Mat> detectedCorners, Mat detectedIds, List<Mat> rejectedCorners, Mat cameraMatrix, Mat distCoeffs, float minRepDistance, float errorCorrectionRate, boolean checkAllOrders) {
        Mat detectedCorners_mat = Converters.vector_Mat_to_Mat(detectedCorners);
        Mat rejectedCorners_mat = Converters.vector_Mat_to_Mat(rejectedCorners);
        long j = image.nativeObj;
        Mat detectedCorners_mat2 = detectedCorners_mat;
        long j2 = j;
        refineDetectedMarkers_2(j2, board.getNativeObjAddr(), detectedCorners_mat.nativeObj, detectedIds.nativeObj, rejectedCorners_mat.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, minRepDistance, errorCorrectionRate, checkAllOrders);
        Mat detectedCorners_mat3 = detectedCorners_mat2;
        Converters.Mat_to_vector_Mat(detectedCorners_mat3, detectedCorners);
        detectedCorners_mat3.release();
        Mat rejectedCorners_mat2 = rejectedCorners_mat;
        Converters.Mat_to_vector_Mat(rejectedCorners_mat2, rejectedCorners);
        rejectedCorners_mat2.release();
    }

    public static void refineDetectedMarkers(Mat image, Board board, List<Mat> detectedCorners, Mat detectedIds, List<Mat> rejectedCorners, Mat cameraMatrix, Mat distCoeffs, float minRepDistance, float errorCorrectionRate) {
        Mat detectedCorners_mat = Converters.vector_Mat_to_Mat(detectedCorners);
        Mat rejectedCorners_mat = Converters.vector_Mat_to_Mat(rejectedCorners);
        Mat rejectedCorners_mat2 = rejectedCorners_mat;
        refineDetectedMarkers_3(image.nativeObj, board.getNativeObjAddr(), detectedCorners_mat.nativeObj, detectedIds.nativeObj, rejectedCorners_mat.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, minRepDistance, errorCorrectionRate);
        Mat detectedCorners_mat2 = detectedCorners_mat;
        Converters.Mat_to_vector_Mat(detectedCorners_mat2, detectedCorners);
        detectedCorners_mat2.release();
        Mat rejectedCorners_mat3 = rejectedCorners_mat2;
        Converters.Mat_to_vector_Mat(rejectedCorners_mat3, rejectedCorners);
        rejectedCorners_mat3.release();
    }

    public static void refineDetectedMarkers(Mat image, Board board, List<Mat> detectedCorners, Mat detectedIds, List<Mat> rejectedCorners, Mat cameraMatrix, Mat distCoeffs, float minRepDistance) {
        Mat detectedCorners_mat = Converters.vector_Mat_to_Mat(detectedCorners);
        Mat rejectedCorners_mat = Converters.vector_Mat_to_Mat(rejectedCorners);
        Mat rejectedCorners_mat2 = rejectedCorners_mat;
        refineDetectedMarkers_4(image.nativeObj, board.getNativeObjAddr(), detectedCorners_mat.nativeObj, detectedIds.nativeObj, rejectedCorners_mat.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, minRepDistance);
        Mat detectedCorners_mat2 = detectedCorners_mat;
        Converters.Mat_to_vector_Mat(detectedCorners_mat2, detectedCorners);
        detectedCorners_mat2.release();
        Mat rejectedCorners_mat3 = rejectedCorners_mat2;
        Converters.Mat_to_vector_Mat(rejectedCorners_mat3, rejectedCorners);
        rejectedCorners_mat3.release();
    }

    public static void refineDetectedMarkers(Mat image, Board board, List<Mat> detectedCorners, Mat detectedIds, List<Mat> rejectedCorners, Mat cameraMatrix, Mat distCoeffs) {
        Mat detectedCorners_mat = Converters.vector_Mat_to_Mat(detectedCorners);
        Mat rejectedCorners_mat = Converters.vector_Mat_to_Mat(rejectedCorners);
        Mat rejectedCorners_mat2 = rejectedCorners_mat;
        refineDetectedMarkers_5(image.nativeObj, board.getNativeObjAddr(), detectedCorners_mat.nativeObj, detectedIds.nativeObj, rejectedCorners_mat.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj);
        Mat detectedCorners_mat2 = detectedCorners_mat;
        Converters.Mat_to_vector_Mat(detectedCorners_mat2, detectedCorners);
        detectedCorners_mat2.release();
        Mat rejectedCorners_mat3 = rejectedCorners_mat2;
        Converters.Mat_to_vector_Mat(rejectedCorners_mat3, rejectedCorners);
        rejectedCorners_mat3.release();
    }

    public static void refineDetectedMarkers(Mat image, Board board, List<Mat> detectedCorners, Mat detectedIds, List<Mat> rejectedCorners, Mat cameraMatrix) {
        Mat detectedCorners_mat = Converters.vector_Mat_to_Mat(detectedCorners);
        Mat rejectedCorners_mat = Converters.vector_Mat_to_Mat(rejectedCorners);
        refineDetectedMarkers_6(image.nativeObj, board.getNativeObjAddr(), detectedCorners_mat.nativeObj, detectedIds.nativeObj, rejectedCorners_mat.nativeObj, cameraMatrix.nativeObj);
        Converters.Mat_to_vector_Mat(detectedCorners_mat, detectedCorners);
        detectedCorners_mat.release();
        Converters.Mat_to_vector_Mat(rejectedCorners_mat, rejectedCorners);
        rejectedCorners_mat.release();
    }

    public static void refineDetectedMarkers(Mat image, Board board, List<Mat> detectedCorners, Mat detectedIds, List<Mat> rejectedCorners) {
        Mat detectedCorners_mat = Converters.vector_Mat_to_Mat(detectedCorners);
        Mat rejectedCorners_mat = Converters.vector_Mat_to_Mat(rejectedCorners);
        refineDetectedMarkers_7(image.nativeObj, board.getNativeObjAddr(), detectedCorners_mat.nativeObj, detectedIds.nativeObj, rejectedCorners_mat.nativeObj);
        Converters.Mat_to_vector_Mat(detectedCorners_mat, detectedCorners);
        detectedCorners_mat.release();
        Converters.Mat_to_vector_Mat(rejectedCorners_mat, rejectedCorners);
        rejectedCorners_mat.release();
    }
}
