package org.opencv.text;

import java.util.ArrayList;
import java.util.List;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfRect;
import org.opencv.utils.Converters;

public class Text {
    public static final int ERFILTER_NM_IHSGrad = 1;
    public static final int ERFILTER_NM_RGBLGrad = 0;
    public static final int ERGROUPING_ORIENTATION_ANY = 1;
    public static final int ERGROUPING_ORIENTATION_HORIZ = 0;
    public static final int OCR_CNN_CLASSIFIER = 1;
    public static final int OCR_DECODER_VITERBI = 0;
    public static final int OCR_KNN_CLASSIFIER = 0;
    public static final int OCR_LEVEL_TEXTLINE = 1;
    public static final int OCR_LEVEL_WORD = 0;
    public static final int OEM_CUBE_ONLY = 1;
    public static final int OEM_DEFAULT = 3;
    public static final int OEM_TESSERACT_CUBE_COMBINED = 2;
    public static final int OEM_TESSERACT_ONLY = 0;
    public static final int PSM_AUTO = 3;
    public static final int PSM_AUTO_ONLY = 2;
    public static final int PSM_AUTO_OSD = 1;
    public static final int PSM_CIRCLE_WORD = 9;
    public static final int PSM_OSD_ONLY = 0;
    public static final int PSM_SINGLE_BLOCK = 6;
    public static final int PSM_SINGLE_BLOCK_VERT_TEXT = 5;
    public static final int PSM_SINGLE_CHAR = 10;
    public static final int PSM_SINGLE_COLUMN = 4;
    public static final int PSM_SINGLE_LINE = 7;
    public static final int PSM_SINGLE_WORD = 8;

    private static native void computeNMChannels_0(long j, long j2, int i);

    private static native void computeNMChannels_1(long j, long j2);

    private static native long createERFilterNM1_0(String str, int i, float f, float f2, float f3, boolean z, float f4);

    private static native long createERFilterNM1_1(String str, int i, float f, float f2, float f3, boolean z);

    private static native long createERFilterNM1_2(String str, int i, float f, float f2, float f3);

    private static native long createERFilterNM1_3(String str, int i, float f, float f2);

    private static native long createERFilterNM1_4(String str, int i, float f);

    private static native long createERFilterNM1_5(String str, int i);

    private static native long createERFilterNM1_6(String str);

    private static native long createERFilterNM2_0(String str, float f);

    private static native long createERFilterNM2_1(String str);

    private static native long createOCRHMMTransitionsTable_0(String str, List<String> list);

    private static native void detectRegions_0(long j, long j2, long j3, long j4, int i, String str, float f);

    private static native void detectRegions_1(long j, long j2, long j3, long j4, int i, String str);

    private static native void detectRegions_2(long j, long j2, long j3, long j4, int i);

    private static native void detectRegions_3(long j, long j2, long j3, long j4);

    private static native void detectRegions_4(long j, long j2, long j3, long j4);

    private static native void erGrouping_0(long j, long j2, long j3, long j4, int i, String str, float f);

    private static native void erGrouping_1(long j, long j2, long j3, long j4, int i, String str);

    private static native void erGrouping_2(long j, long j2, long j3, long j4, int i);

    private static native void erGrouping_3(long j, long j2, long j3, long j4);

    public static Mat createOCRHMMTransitionsTable(String vocabulary, List<String> lexicon) {
        return new Mat(createOCRHMMTransitionsTable_0(vocabulary, lexicon));
    }

    public static ERFilter createERFilterNM1(String filename, int thresholdDelta, float minArea, float maxArea, float minProbability, boolean nonMaxSuppression, float minProbabilityDiff) {
        return ERFilter.__fromPtr__(createERFilterNM1_0(filename, thresholdDelta, minArea, maxArea, minProbability, nonMaxSuppression, minProbabilityDiff));
    }

    public static ERFilter createERFilterNM1(String filename, int thresholdDelta, float minArea, float maxArea, float minProbability, boolean nonMaxSuppression) {
        return ERFilter.__fromPtr__(createERFilterNM1_1(filename, thresholdDelta, minArea, maxArea, minProbability, nonMaxSuppression));
    }

    public static ERFilter createERFilterNM1(String filename, int thresholdDelta, float minArea, float maxArea, float minProbability) {
        return ERFilter.__fromPtr__(createERFilterNM1_2(filename, thresholdDelta, minArea, maxArea, minProbability));
    }

    public static ERFilter createERFilterNM1(String filename, int thresholdDelta, float minArea, float maxArea) {
        return ERFilter.__fromPtr__(createERFilterNM1_3(filename, thresholdDelta, minArea, maxArea));
    }

    public static ERFilter createERFilterNM1(String filename, int thresholdDelta, float minArea) {
        return ERFilter.__fromPtr__(createERFilterNM1_4(filename, thresholdDelta, minArea));
    }

    public static ERFilter createERFilterNM1(String filename, int thresholdDelta) {
        return ERFilter.__fromPtr__(createERFilterNM1_5(filename, thresholdDelta));
    }

    public static ERFilter createERFilterNM1(String filename) {
        return ERFilter.__fromPtr__(createERFilterNM1_6(filename));
    }

    public static ERFilter createERFilterNM2(String filename, float minProbability) {
        return ERFilter.__fromPtr__(createERFilterNM2_0(filename, minProbability));
    }

    public static ERFilter createERFilterNM2(String filename) {
        return ERFilter.__fromPtr__(createERFilterNM2_1(filename));
    }

    public static void computeNMChannels(Mat _src, List<Mat> _channels, int _mode) {
        Mat _channels_mat = new Mat();
        computeNMChannels_0(_src.nativeObj, _channels_mat.nativeObj, _mode);
        Converters.Mat_to_vector_Mat(_channels_mat, _channels);
        _channels_mat.release();
    }

    public static void computeNMChannels(Mat _src, List<Mat> _channels) {
        Mat _channels_mat = new Mat();
        computeNMChannels_1(_src.nativeObj, _channels_mat.nativeObj);
        Converters.Mat_to_vector_Mat(_channels_mat, _channels);
        _channels_mat.release();
    }

    public static void detectRegions(Mat image, ERFilter er_filter1, ERFilter er_filter2, MatOfRect groups_rects, int method, String filename, float minProbability) {
        detectRegions_0(image.nativeObj, er_filter1.getNativeObjAddr(), er_filter2.getNativeObjAddr(), groups_rects.nativeObj, method, filename, minProbability);
    }

    public static void detectRegions(Mat image, ERFilter er_filter1, ERFilter er_filter2, MatOfRect groups_rects, int method, String filename) {
        detectRegions_1(image.nativeObj, er_filter1.getNativeObjAddr(), er_filter2.getNativeObjAddr(), groups_rects.nativeObj, method, filename);
    }

    public static void detectRegions(Mat image, ERFilter er_filter1, ERFilter er_filter2, MatOfRect groups_rects, int method) {
        detectRegions_2(image.nativeObj, er_filter1.getNativeObjAddr(), er_filter2.getNativeObjAddr(), groups_rects.nativeObj, method);
    }

    public static void detectRegions(Mat image, ERFilter er_filter1, ERFilter er_filter2, MatOfRect groups_rects) {
        detectRegions_3(image.nativeObj, er_filter1.getNativeObjAddr(), er_filter2.getNativeObjAddr(), groups_rects.nativeObj);
    }

    public static void detectRegions(Mat image, ERFilter er_filter1, ERFilter er_filter2, List<MatOfPoint> regions) {
        Mat regions_mat = new Mat();
        detectRegions_4(image.nativeObj, er_filter1.getNativeObjAddr(), er_filter2.getNativeObjAddr(), regions_mat.nativeObj);
        Converters.Mat_to_vector_vector_Point(regions_mat, regions);
        regions_mat.release();
    }

    public static void erGrouping(Mat image, Mat channel, List<MatOfPoint> regions, MatOfRect groups_rects, int method, String filename, float minProbablity) {
        List<MatOfPoint> list = regions;
        erGrouping_0(image.nativeObj, channel.nativeObj, Converters.vector_vector_Point_to_Mat(list, new ArrayList<>(list != null ? regions.size() : 0)).nativeObj, groups_rects.nativeObj, method, filename, minProbablity);
    }

    public static void erGrouping(Mat image, Mat channel, List<MatOfPoint> regions, MatOfRect groups_rects, int method, String filename) {
        List<MatOfPoint> list = regions;
        erGrouping_1(image.nativeObj, channel.nativeObj, Converters.vector_vector_Point_to_Mat(list, new ArrayList<>(list != null ? regions.size() : 0)).nativeObj, groups_rects.nativeObj, method, filename);
    }

    public static void erGrouping(Mat image, Mat channel, List<MatOfPoint> regions, MatOfRect groups_rects, int method) {
        List<MatOfPoint> list = regions;
        erGrouping_2(image.nativeObj, channel.nativeObj, Converters.vector_vector_Point_to_Mat(list, new ArrayList<>(list != null ? regions.size() : 0)).nativeObj, groups_rects.nativeObj, method);
    }

    public static void erGrouping(Mat image, Mat channel, List<MatOfPoint> regions, MatOfRect groups_rects) {
        erGrouping_3(image.nativeObj, channel.nativeObj, Converters.vector_vector_Point_to_Mat(regions, new ArrayList<>(regions != null ? regions.size() : 0)).nativeObj, groups_rects.nativeObj);
    }
}
