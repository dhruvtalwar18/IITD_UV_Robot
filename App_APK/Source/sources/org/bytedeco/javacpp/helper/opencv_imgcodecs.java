package org.bytedeco.javacpp.helper;

import org.bytedeco.javacpp.helper.opencv_core;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.opencv_imgproc;

public class opencv_imgcodecs extends org.bytedeco.javacpp.presets.opencv_imgcodecs {
    public static opencv_core.IplImage cvLoadImage(String filename) {
        return cvLoadImage(filename, 1);
    }

    public static opencv_core.IplImage cvLoadImage(String filename, int iscolor) {
        return new opencv_core.IplImage(opencv_core.cvClone(opencv_core.cvIplImage(org.bytedeco.javacpp.opencv_imgcodecs.imread(filename, iscolor))));
    }

    public static int cvSaveImage(String filename, opencv_core.CvArr arr) {
        return cvSaveImage(filename, arr, (int[]) null);
    }

    public static int cvSaveImage(String filename, opencv_core.CvArr arr, int[] params) {
        return org.bytedeco.javacpp.opencv_imgcodecs.imwrite(filename, org.bytedeco.javacpp.opencv_core.cvarrToMat(arr), params) ? 1 : 0;
    }

    public static opencv_core.IplImage cvLoadImageBGRA(String filename) {
        opencv_core.IplImage imageBGR = cvLoadImage(filename, 1);
        if (imageBGR == null) {
            return null;
        }
        opencv_core.IplImage imageBGRA = org.bytedeco.javacpp.opencv_core.cvCreateImage(org.bytedeco.javacpp.opencv_core.cvGetSize(imageBGR), imageBGR.depth(), 4);
        opencv_imgproc.cvCvtColor(imageBGR, imageBGRA, 0);
        org.bytedeco.javacpp.opencv_core.cvReleaseImage(imageBGR);
        return imageBGRA;
    }

    public static opencv_core.IplImage cvLoadImageRGBA(String filename) {
        opencv_core.IplImage imageBGR = cvLoadImage(filename, 1);
        if (imageBGR == null) {
            return null;
        }
        opencv_core.IplImage imageRGBA = org.bytedeco.javacpp.opencv_core.cvCreateImage(org.bytedeco.javacpp.opencv_core.cvGetSize(imageBGR), imageBGR.depth(), 4);
        opencv_imgproc.cvCvtColor(imageBGR, imageRGBA, 2);
        org.bytedeco.javacpp.opencv_core.cvReleaseImage(imageBGR);
        return imageRGBA;
    }
}
