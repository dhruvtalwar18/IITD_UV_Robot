package org.opencv.dnn;

import java.util.List;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfRect;
import org.opencv.core.MatOfRect2d;
import org.opencv.core.MatOfRotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.utils.Converters;

public class Dnn {
    public static final int DNN_BACKEND_DEFAULT = 0;
    public static final int DNN_BACKEND_HALIDE = 1;
    public static final int DNN_BACKEND_INFERENCE_ENGINE = 2;
    public static final int DNN_BACKEND_OPENCV = 3;
    public static final int DNN_BACKEND_VKCOM = 4;
    public static final int DNN_TARGET_CPU = 0;
    public static final int DNN_TARGET_FPGA = 5;
    public static final int DNN_TARGET_MYRIAD = 3;
    public static final int DNN_TARGET_OPENCL = 1;
    public static final int DNN_TARGET_OPENCL_FP16 = 2;
    public static final int DNN_TARGET_VULKAN = 4;

    private static native void NMSBoxesRotated_0(long j, long j2, float f, float f2, long j3, float f3, int i);

    private static native void NMSBoxesRotated_1(long j, long j2, float f, float f2, long j3, float f3);

    private static native void NMSBoxesRotated_2(long j, long j2, float f, float f2, long j3);

    private static native void NMSBoxes_0(long j, long j2, float f, float f2, long j3, float f3, int i);

    private static native void NMSBoxes_1(long j, long j2, float f, float f2, long j3, float f3);

    private static native void NMSBoxes_2(long j, long j2, float f, float f2, long j3);

    private static native void NMSBoxes_3(long j, long j2, float f, float f2, long j3, float f3, int i);

    private static native void NMSBoxes_4(long j, long j2, float f, float f2, long j3, float f3);

    private static native void NMSBoxes_5(long j, long j2, float f, float f2, long j3);

    private static native long blobFromImage_0(long j, double d, double d2, double d3, double d4, double d5, double d6, double d7, boolean z, boolean z2, int i);

    private static native long blobFromImage_1(long j, double d, double d2, double d3, double d4, double d5, double d6, double d7, boolean z, boolean z2);

    private static native long blobFromImage_2(long j, double d, double d2, double d3, double d4, double d5, double d6, double d7, boolean z);

    private static native long blobFromImage_3(long j, double d, double d2, double d3, double d4, double d5, double d6, double d7);

    private static native long blobFromImage_4(long j, double d, double d2, double d3);

    private static native long blobFromImage_5(long j, double d);

    private static native long blobFromImage_6(long j);

    private static native long blobFromImages_0(long j, double d, double d2, double d3, double d4, double d5, double d6, double d7, boolean z, boolean z2, int i);

    private static native long blobFromImages_1(long j, double d, double d2, double d3, double d4, double d5, double d6, double d7, boolean z, boolean z2);

    private static native long blobFromImages_2(long j, double d, double d2, double d3, double d4, double d5, double d6, double d7, boolean z);

    private static native long blobFromImages_3(long j, double d, double d2, double d3, double d4, double d5, double d6, double d7);

    private static native long blobFromImages_4(long j, double d, double d2, double d3);

    private static native long blobFromImages_5(long j, double d);

    private static native long blobFromImages_6(long j);

    private static native void imagesFromBlob_0(long j, long j2);

    private static native long readNetFromCaffe_0(String str, String str2);

    private static native long readNetFromCaffe_1(String str);

    private static native long readNetFromCaffe_2(long j, long j2);

    private static native long readNetFromCaffe_3(long j);

    private static native long readNetFromDarknet_0(String str, String str2);

    private static native long readNetFromDarknet_1(String str);

    private static native long readNetFromDarknet_2(long j, long j2);

    private static native long readNetFromDarknet_3(long j);

    private static native long readNetFromModelOptimizer_0(String str, String str2);

    private static native long readNetFromONNX_0(String str);

    private static native long readNetFromTensorflow_0(String str, String str2);

    private static native long readNetFromTensorflow_1(String str);

    private static native long readNetFromTensorflow_2(long j, long j2);

    private static native long readNetFromTensorflow_3(long j);

    private static native long readNetFromTorch_0(String str, boolean z, boolean z2);

    private static native long readNetFromTorch_1(String str, boolean z);

    private static native long readNetFromTorch_2(String str);

    private static native long readNet_0(String str, long j, long j2);

    private static native long readNet_1(String str, long j);

    private static native long readNet_2(String str, String str2, String str3);

    private static native long readNet_3(String str, String str2);

    private static native long readNet_4(String str);

    private static native long readTensorFromONNX_0(String str);

    private static native long readTorchBlob_0(String str, boolean z);

    private static native long readTorchBlob_1(String str);

    private static native void resetMyriadDevice_0();

    private static native void shrinkCaffeModel_0(String str, String str2, List<String> list);

    private static native void shrinkCaffeModel_1(String str, String str2);

    private static native void writeTextGraph_0(String str, String str2);

    public static Mat blobFromImage(Mat image, double scalefactor, Size size, Scalar mean, boolean swapRB, boolean crop, int ddepth) {
        Size size2 = size;
        Scalar scalar = mean;
        return new Mat(blobFromImage_0(image.nativeObj, scalefactor, size2.width, size2.height, scalar.val[0], scalar.val[1], scalar.val[2], scalar.val[3], swapRB, crop, ddepth));
    }

    public static Mat blobFromImage(Mat image, double scalefactor, Size size, Scalar mean, boolean swapRB, boolean crop) {
        Size size2 = size;
        Scalar scalar = mean;
        return new Mat(blobFromImage_1(image.nativeObj, scalefactor, size2.width, size2.height, scalar.val[0], scalar.val[1], scalar.val[2], scalar.val[3], swapRB, crop));
    }

    public static Mat blobFromImage(Mat image, double scalefactor, Size size, Scalar mean, boolean swapRB) {
        Size size2 = size;
        Scalar scalar = mean;
        return new Mat(blobFromImage_2(image.nativeObj, scalefactor, size2.width, size2.height, scalar.val[0], scalar.val[1], scalar.val[2], scalar.val[3], swapRB));
    }

    public static Mat blobFromImage(Mat image, double scalefactor, Size size, Scalar mean) {
        Size size2 = size;
        Scalar scalar = mean;
        return new Mat(blobFromImage_3(image.nativeObj, scalefactor, size2.width, size2.height, scalar.val[0], scalar.val[1], scalar.val[2], scalar.val[3]));
    }

    public static Mat blobFromImage(Mat image, double scalefactor, Size size) {
        return new Mat(blobFromImage_4(image.nativeObj, scalefactor, size.width, size.height));
    }

    public static Mat blobFromImage(Mat image, double scalefactor) {
        return new Mat(blobFromImage_5(image.nativeObj, scalefactor));
    }

    public static Mat blobFromImage(Mat image) {
        return new Mat(blobFromImage_6(image.nativeObj));
    }

    public static Mat blobFromImages(List<Mat> images, double scalefactor, Size size, Scalar mean, boolean swapRB, boolean crop, int ddepth) {
        Size size2 = size;
        Scalar scalar = mean;
        Mat images_mat = Converters.vector_Mat_to_Mat(images);
        Mat mat = images_mat;
        return new Mat(blobFromImages_0(images_mat.nativeObj, scalefactor, size2.width, size2.height, scalar.val[0], scalar.val[1], scalar.val[2], scalar.val[3], swapRB, crop, ddepth));
    }

    public static Mat blobFromImages(List<Mat> images, double scalefactor, Size size, Scalar mean, boolean swapRB, boolean crop) {
        Size size2 = size;
        Scalar scalar = mean;
        Mat images_mat = Converters.vector_Mat_to_Mat(images);
        Mat mat = images_mat;
        return new Mat(blobFromImages_1(images_mat.nativeObj, scalefactor, size2.width, size2.height, scalar.val[0], scalar.val[1], scalar.val[2], scalar.val[3], swapRB, crop));
    }

    public static Mat blobFromImages(List<Mat> images, double scalefactor, Size size, Scalar mean, boolean swapRB) {
        Size size2 = size;
        Scalar scalar = mean;
        Mat images_mat = Converters.vector_Mat_to_Mat(images);
        Mat mat = images_mat;
        return new Mat(blobFromImages_2(images_mat.nativeObj, scalefactor, size2.width, size2.height, scalar.val[0], scalar.val[1], scalar.val[2], scalar.val[3], swapRB));
    }

    public static Mat blobFromImages(List<Mat> images, double scalefactor, Size size, Scalar mean) {
        Size size2 = size;
        Scalar scalar = mean;
        return new Mat(blobFromImages_3(Converters.vector_Mat_to_Mat(images).nativeObj, scalefactor, size2.width, size2.height, scalar.val[0], scalar.val[1], scalar.val[2], scalar.val[3]));
    }

    public static Mat blobFromImages(List<Mat> images, double scalefactor, Size size) {
        return new Mat(blobFromImages_4(Converters.vector_Mat_to_Mat(images).nativeObj, scalefactor, size.width, size.height));
    }

    public static Mat blobFromImages(List<Mat> images, double scalefactor) {
        return new Mat(blobFromImages_5(Converters.vector_Mat_to_Mat(images).nativeObj, scalefactor));
    }

    public static Mat blobFromImages(List<Mat> images) {
        return new Mat(blobFromImages_6(Converters.vector_Mat_to_Mat(images).nativeObj));
    }

    public static Mat readTensorFromONNX(String path) {
        return new Mat(readTensorFromONNX_0(path));
    }

    public static Mat readTorchBlob(String filename, boolean isBinary) {
        return new Mat(readTorchBlob_0(filename, isBinary));
    }

    public static Mat readTorchBlob(String filename) {
        return new Mat(readTorchBlob_1(filename));
    }

    public static Net readNet(String framework, MatOfByte bufferModel, MatOfByte bufferConfig) {
        return new Net(readNet_0(framework, bufferModel.nativeObj, bufferConfig.nativeObj));
    }

    public static Net readNet(String framework, MatOfByte bufferModel) {
        return new Net(readNet_1(framework, bufferModel.nativeObj));
    }

    public static Net readNet(String model, String config, String framework) {
        return new Net(readNet_2(model, config, framework));
    }

    public static Net readNet(String model, String config) {
        return new Net(readNet_3(model, config));
    }

    public static Net readNet(String model) {
        return new Net(readNet_4(model));
    }

    public static Net readNetFromCaffe(String prototxt, String caffeModel) {
        return new Net(readNetFromCaffe_0(prototxt, caffeModel));
    }

    public static Net readNetFromCaffe(String prototxt) {
        return new Net(readNetFromCaffe_1(prototxt));
    }

    public static Net readNetFromCaffe(MatOfByte bufferProto, MatOfByte bufferModel) {
        return new Net(readNetFromCaffe_2(bufferProto.nativeObj, bufferModel.nativeObj));
    }

    public static Net readNetFromCaffe(MatOfByte bufferProto) {
        return new Net(readNetFromCaffe_3(bufferProto.nativeObj));
    }

    public static Net readNetFromDarknet(String cfgFile, String darknetModel) {
        return new Net(readNetFromDarknet_0(cfgFile, darknetModel));
    }

    public static Net readNetFromDarknet(String cfgFile) {
        return new Net(readNetFromDarknet_1(cfgFile));
    }

    public static Net readNetFromDarknet(MatOfByte bufferCfg, MatOfByte bufferModel) {
        return new Net(readNetFromDarknet_2(bufferCfg.nativeObj, bufferModel.nativeObj));
    }

    public static Net readNetFromDarknet(MatOfByte bufferCfg) {
        return new Net(readNetFromDarknet_3(bufferCfg.nativeObj));
    }

    public static Net readNetFromModelOptimizer(String xml, String bin) {
        return new Net(readNetFromModelOptimizer_0(xml, bin));
    }

    public static Net readNetFromONNX(String onnxFile) {
        return new Net(readNetFromONNX_0(onnxFile));
    }

    public static Net readNetFromTensorflow(String model, String config) {
        return new Net(readNetFromTensorflow_0(model, config));
    }

    public static Net readNetFromTensorflow(String model) {
        return new Net(readNetFromTensorflow_1(model));
    }

    public static Net readNetFromTensorflow(MatOfByte bufferModel, MatOfByte bufferConfig) {
        return new Net(readNetFromTensorflow_2(bufferModel.nativeObj, bufferConfig.nativeObj));
    }

    public static Net readNetFromTensorflow(MatOfByte bufferModel) {
        return new Net(readNetFromTensorflow_3(bufferModel.nativeObj));
    }

    public static Net readNetFromTorch(String model, boolean isBinary, boolean evaluate) {
        return new Net(readNetFromTorch_0(model, isBinary, evaluate));
    }

    public static Net readNetFromTorch(String model, boolean isBinary) {
        return new Net(readNetFromTorch_1(model, isBinary));
    }

    public static Net readNetFromTorch(String model) {
        return new Net(readNetFromTorch_2(model));
    }

    public static void NMSBoxes(MatOfRect bboxes, MatOfFloat scores, float score_threshold, float nms_threshold, MatOfInt indices, float eta, int top_k) {
        NMSBoxes_0(bboxes.nativeObj, scores.nativeObj, score_threshold, nms_threshold, indices.nativeObj, eta, top_k);
    }

    public static void NMSBoxes(MatOfRect bboxes, MatOfFloat scores, float score_threshold, float nms_threshold, MatOfInt indices, float eta) {
        NMSBoxes_1(bboxes.nativeObj, scores.nativeObj, score_threshold, nms_threshold, indices.nativeObj, eta);
    }

    public static void NMSBoxes(MatOfRect bboxes, MatOfFloat scores, float score_threshold, float nms_threshold, MatOfInt indices) {
        NMSBoxes_2(bboxes.nativeObj, scores.nativeObj, score_threshold, nms_threshold, indices.nativeObj);
    }

    public static void NMSBoxes(MatOfRect2d bboxes, MatOfFloat scores, float score_threshold, float nms_threshold, MatOfInt indices, float eta, int top_k) {
        NMSBoxes_3(bboxes.nativeObj, scores.nativeObj, score_threshold, nms_threshold, indices.nativeObj, eta, top_k);
    }

    public static void NMSBoxes(MatOfRect2d bboxes, MatOfFloat scores, float score_threshold, float nms_threshold, MatOfInt indices, float eta) {
        NMSBoxes_4(bboxes.nativeObj, scores.nativeObj, score_threshold, nms_threshold, indices.nativeObj, eta);
    }

    public static void NMSBoxes(MatOfRect2d bboxes, MatOfFloat scores, float score_threshold, float nms_threshold, MatOfInt indices) {
        NMSBoxes_5(bboxes.nativeObj, scores.nativeObj, score_threshold, nms_threshold, indices.nativeObj);
    }

    public static void NMSBoxesRotated(MatOfRotatedRect bboxes, MatOfFloat scores, float score_threshold, float nms_threshold, MatOfInt indices, float eta, int top_k) {
        NMSBoxesRotated_0(bboxes.nativeObj, scores.nativeObj, score_threshold, nms_threshold, indices.nativeObj, eta, top_k);
    }

    public static void NMSBoxesRotated(MatOfRotatedRect bboxes, MatOfFloat scores, float score_threshold, float nms_threshold, MatOfInt indices, float eta) {
        NMSBoxesRotated_1(bboxes.nativeObj, scores.nativeObj, score_threshold, nms_threshold, indices.nativeObj, eta);
    }

    public static void NMSBoxesRotated(MatOfRotatedRect bboxes, MatOfFloat scores, float score_threshold, float nms_threshold, MatOfInt indices) {
        NMSBoxesRotated_2(bboxes.nativeObj, scores.nativeObj, score_threshold, nms_threshold, indices.nativeObj);
    }

    public static void imagesFromBlob(Mat blob_, List<Mat> images_) {
        Mat images__mat = new Mat();
        imagesFromBlob_0(blob_.nativeObj, images__mat.nativeObj);
        Converters.Mat_to_vector_Mat(images__mat, images_);
        images__mat.release();
    }

    public static void resetMyriadDevice() {
        resetMyriadDevice_0();
    }

    public static void shrinkCaffeModel(String src, String dst, List<String> layersTypes) {
        shrinkCaffeModel_0(src, dst, layersTypes);
    }

    public static void shrinkCaffeModel(String src, String dst) {
        shrinkCaffeModel_1(src, dst);
    }

    public static void writeTextGraph(String model, String output) {
        writeTextGraph_0(model, output);
    }
}
