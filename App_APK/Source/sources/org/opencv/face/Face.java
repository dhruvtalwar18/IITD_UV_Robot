package org.opencv.face;

import java.util.ArrayList;
import java.util.List;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Scalar;
import org.opencv.utils.Converters;

public class Face {
    private static native long createFacemarkAAM_0();

    private static native long createFacemarkKazemi_0();

    private static native long createFacemarkLBF_0();

    private static native void drawFacemarks_0(long j, long j2, double d, double d2, double d3, double d4);

    private static native void drawFacemarks_1(long j, long j2);

    private static native boolean getFacesHAAR_0(long j, long j2, String str);

    private static native boolean loadDatasetList_0(String str, String str2, List<String> list, List<String> list2);

    private static native boolean loadFacePoints_0(String str, long j, float f);

    private static native boolean loadFacePoints_1(String str, long j);

    private static native boolean loadTrainingData_0(String str, List<String> list, long j, char c, float f);

    private static native boolean loadTrainingData_1(String str, List<String> list, long j, char c);

    private static native boolean loadTrainingData_2(String str, List<String> list, long j);

    private static native boolean loadTrainingData_3(String str, String str2, List<String> list, long j, float f);

    private static native boolean loadTrainingData_4(String str, String str2, List<String> list, long j);

    private static native boolean loadTrainingData_5(List<String> list, long j, List<String> list2);

    public static Facemark createFacemarkAAM() {
        return Facemark.__fromPtr__(createFacemarkAAM_0());
    }

    public static Facemark createFacemarkKazemi() {
        return Facemark.__fromPtr__(createFacemarkKazemi_0());
    }

    public static Facemark createFacemarkLBF() {
        return Facemark.__fromPtr__(createFacemarkLBF_0());
    }

    public static boolean getFacesHAAR(Mat image, Mat faces, String face_cascade_name) {
        return getFacesHAAR_0(image.nativeObj, faces.nativeObj, face_cascade_name);
    }

    public static boolean loadDatasetList(String imageList, String annotationList, List<String> images, List<String> annotations) {
        return loadDatasetList_0(imageList, annotationList, images, annotations);
    }

    public static boolean loadFacePoints(String filename, Mat points, float offset) {
        return loadFacePoints_0(filename, points.nativeObj, offset);
    }

    public static boolean loadFacePoints(String filename, Mat points) {
        return loadFacePoints_1(filename, points.nativeObj);
    }

    public static boolean loadTrainingData(String filename, List<String> images, Mat facePoints, char delim, float offset) {
        return loadTrainingData_0(filename, images, facePoints.nativeObj, delim, offset);
    }

    public static boolean loadTrainingData(String filename, List<String> images, Mat facePoints, char delim) {
        return loadTrainingData_1(filename, images, facePoints.nativeObj, delim);
    }

    public static boolean loadTrainingData(String filename, List<String> images, Mat facePoints) {
        return loadTrainingData_2(filename, images, facePoints.nativeObj);
    }

    public static boolean loadTrainingData(String imageList, String groundTruth, List<String> images, Mat facePoints, float offset) {
        return loadTrainingData_3(imageList, groundTruth, images, facePoints.nativeObj, offset);
    }

    public static boolean loadTrainingData(String imageList, String groundTruth, List<String> images, Mat facePoints) {
        return loadTrainingData_4(imageList, groundTruth, images, facePoints.nativeObj);
    }

    public static boolean loadTrainingData(List<String> filename, List<MatOfPoint2f> trainlandmarks, List<String> trainimages) {
        return loadTrainingData_5(filename, Converters.vector_vector_Point2f_to_Mat(trainlandmarks, new ArrayList<>(trainlandmarks != null ? trainlandmarks.size() : 0)).nativeObj, trainimages);
    }

    public static void drawFacemarks(Mat image, Mat points, Scalar color) {
        drawFacemarks_0(image.nativeObj, points.nativeObj, color.val[0], color.val[1], color.val[2], color.val[3]);
    }

    public static void drawFacemarks(Mat image, Mat points) {
        drawFacemarks_1(image.nativeObj, points.nativeObj);
    }
}
