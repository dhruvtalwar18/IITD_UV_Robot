package org.bytedeco.javacpp;

import java.nio.IntBuffer;
import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Const;
import org.bytedeco.javacpp.annotation.Name;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.annotation.NoOffset;
import org.bytedeco.javacpp.annotation.StdVector;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.presets.opencv_core;

public class opencv_aruco extends org.bytedeco.javacpp.presets.opencv_aruco {
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

    @Namespace("cv::aruco")
    public static native double calibrateCameraAruco(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4);

    @Namespace("cv::aruco")
    public static native double calibrateCameraAruco(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector2, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector3, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    public static native double calibrateCameraAruco(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4);

    @Namespace("cv::aruco")
    public static native double calibrateCameraAruco(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector2, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector3, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    public static native double calibrateCameraAruco(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4);

    @Namespace("cv::aruco")
    public static native double calibrateCameraAruco(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector2, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector3, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    public static native double calibrateCameraAruco(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4);

    @Namespace("cv::aruco")
    public static native double calibrateCameraAruco(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector2, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector3, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    public static native double calibrateCameraAruco(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4);

    @Namespace("cv::aruco")
    public static native double calibrateCameraAruco(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector2, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector3, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    public static native double calibrateCameraAruco(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4);

    @Namespace("cv::aruco")
    public static native double calibrateCameraAruco(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector2, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector3, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    public static native double calibrateCameraAruco(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4);

    @Namespace("cv::aruco")
    public static native double calibrateCameraAruco(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector2, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector3, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    public static native double calibrateCameraAruco(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4);

    @Namespace("cv::aruco")
    public static native double calibrateCameraAruco(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector2, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector3, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    public static native double calibrateCameraAruco(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4);

    @Namespace("cv::aruco")
    public static native double calibrateCameraAruco(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector2, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector3, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraAruco"})
    public static native double calibrateCameraArucoExtended(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByVal opencv_core.GpuMatVector gpuMatVector3, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6, @ByVal opencv_core.GpuMat gpuMat7);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraAruco"})
    public static native double calibrateCameraArucoExtended(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByVal opencv_core.GpuMatVector gpuMatVector3, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6, @ByVal opencv_core.GpuMat gpuMat7, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraAruco"})
    public static native double calibrateCameraArucoExtended(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByVal opencv_core.GpuMatVector gpuMatVector3, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, @ByVal opencv_core.Mat mat7);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraAruco"})
    public static native double calibrateCameraArucoExtended(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByVal opencv_core.GpuMatVector gpuMatVector3, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, @ByVal opencv_core.Mat mat7, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraAruco"})
    public static native double calibrateCameraArucoExtended(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByVal opencv_core.GpuMatVector gpuMatVector3, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6, @ByVal opencv_core.UMat uMat7);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraAruco"})
    public static native double calibrateCameraArucoExtended(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByVal opencv_core.GpuMatVector gpuMatVector3, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6, @ByVal opencv_core.UMat uMat7, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraAruco"})
    public static native double calibrateCameraArucoExtended(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.MatVector matVector3, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6, @ByVal opencv_core.GpuMat gpuMat7);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraAruco"})
    public static native double calibrateCameraArucoExtended(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.MatVector matVector3, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6, @ByVal opencv_core.GpuMat gpuMat7, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraAruco"})
    public static native double calibrateCameraArucoExtended(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.MatVector matVector3, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, @ByVal opencv_core.Mat mat7);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraAruco"})
    public static native double calibrateCameraArucoExtended(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.MatVector matVector3, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, @ByVal opencv_core.Mat mat7, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraAruco"})
    public static native double calibrateCameraArucoExtended(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.MatVector matVector3, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6, @ByVal opencv_core.UMat uMat7);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraAruco"})
    public static native double calibrateCameraArucoExtended(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.MatVector matVector3, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6, @ByVal opencv_core.UMat uMat7, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraAruco"})
    public static native double calibrateCameraArucoExtended(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.UMatVector uMatVector2, @ByVal opencv_core.UMatVector uMatVector3, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6, @ByVal opencv_core.GpuMat gpuMat7);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraAruco"})
    public static native double calibrateCameraArucoExtended(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.UMatVector uMatVector2, @ByVal opencv_core.UMatVector uMatVector3, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6, @ByVal opencv_core.GpuMat gpuMat7, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraAruco"})
    public static native double calibrateCameraArucoExtended(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.UMatVector uMatVector2, @ByVal opencv_core.UMatVector uMatVector3, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, @ByVal opencv_core.Mat mat7);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraAruco"})
    public static native double calibrateCameraArucoExtended(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.UMatVector uMatVector2, @ByVal opencv_core.UMatVector uMatVector3, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, @ByVal opencv_core.Mat mat7, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraAruco"})
    public static native double calibrateCameraArucoExtended(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMatVector uMatVector2, @ByVal opencv_core.UMatVector uMatVector3, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6, @ByVal opencv_core.UMat uMat7);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraAruco"})
    public static native double calibrateCameraArucoExtended(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMatVector uMatVector2, @ByVal opencv_core.UMatVector uMatVector3, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6, @ByVal opencv_core.UMat uMat7, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    public static native double calibrateCameraCharuco(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::aruco")
    public static native double calibrateCameraCharuco(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector3, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector4, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    public static native double calibrateCameraCharuco(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::aruco")
    public static native double calibrateCameraCharuco(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector3, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector4, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    public static native double calibrateCameraCharuco(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::aruco")
    public static native double calibrateCameraCharuco(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector3, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector4, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    public static native double calibrateCameraCharuco(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::aruco")
    public static native double calibrateCameraCharuco(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector3, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector4, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    public static native double calibrateCameraCharuco(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::aruco")
    public static native double calibrateCameraCharuco(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector3, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector4, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    public static native double calibrateCameraCharuco(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::aruco")
    public static native double calibrateCameraCharuco(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector3, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector4, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    public static native double calibrateCameraCharuco(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::aruco")
    public static native double calibrateCameraCharuco(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector3, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector4, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    public static native double calibrateCameraCharuco(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::aruco")
    public static native double calibrateCameraCharuco(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector3, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector4, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    public static native double calibrateCameraCharuco(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::aruco")
    public static native double calibrateCameraCharuco(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector3, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector4, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraCharuco"})
    public static native double calibrateCameraCharucoExtended(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMatVector gpuMatVector3, @ByVal opencv_core.GpuMatVector gpuMatVector4, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraCharuco"})
    public static native double calibrateCameraCharucoExtended(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMatVector gpuMatVector3, @ByVal opencv_core.GpuMatVector gpuMatVector4, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraCharuco"})
    public static native double calibrateCameraCharucoExtended(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.GpuMatVector gpuMatVector3, @ByVal opencv_core.GpuMatVector gpuMatVector4, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraCharuco"})
    public static native double calibrateCameraCharucoExtended(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.GpuMatVector gpuMatVector3, @ByVal opencv_core.GpuMatVector gpuMatVector4, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraCharuco"})
    public static native double calibrateCameraCharucoExtended(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.GpuMatVector gpuMatVector3, @ByVal opencv_core.GpuMatVector gpuMatVector4, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraCharuco"})
    public static native double calibrateCameraCharucoExtended(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.GpuMatVector gpuMatVector3, @ByVal opencv_core.GpuMatVector gpuMatVector4, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraCharuco"})
    public static native double calibrateCameraCharucoExtended(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.MatVector matVector3, @ByVal opencv_core.MatVector matVector4, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraCharuco"})
    public static native double calibrateCameraCharucoExtended(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.MatVector matVector3, @ByVal opencv_core.MatVector matVector4, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraCharuco"})
    public static native double calibrateCameraCharucoExtended(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.MatVector matVector3, @ByVal opencv_core.MatVector matVector4, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraCharuco"})
    public static native double calibrateCameraCharucoExtended(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.MatVector matVector3, @ByVal opencv_core.MatVector matVector4, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraCharuco"})
    public static native double calibrateCameraCharucoExtended(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.MatVector matVector3, @ByVal opencv_core.MatVector matVector4, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraCharuco"})
    public static native double calibrateCameraCharucoExtended(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.MatVector matVector3, @ByVal opencv_core.MatVector matVector4, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraCharuco"})
    public static native double calibrateCameraCharucoExtended(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.UMatVector uMatVector3, @ByVal opencv_core.UMatVector uMatVector4, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraCharuco"})
    public static native double calibrateCameraCharucoExtended(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.UMatVector uMatVector3, @ByVal opencv_core.UMatVector uMatVector4, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraCharuco"})
    public static native double calibrateCameraCharucoExtended(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.UMatVector uMatVector3, @ByVal opencv_core.UMatVector uMatVector4, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraCharuco"})
    public static native double calibrateCameraCharucoExtended(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.UMatVector uMatVector3, @ByVal opencv_core.UMatVector uMatVector4, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraCharuco"})
    public static native double calibrateCameraCharucoExtended(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMatVector uMatVector3, @ByVal opencv_core.UMatVector uMatVector4, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5);

    @Namespace("cv::aruco")
    @Name({"calibrateCameraCharuco"})
    public static native double calibrateCameraCharucoExtended(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMatVector uMatVector3, @ByVal opencv_core.UMatVector uMatVector4, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::aruco")
    @opencv_core.Ptr
    @Name({"generateCustomDictionary"})
    public static native Dictionary custom_dictionary(int i, int i2);

    @Namespace("cv::aruco")
    @opencv_core.Ptr
    @Name({"generateCustomDictionary"})
    public static native Dictionary custom_dictionary(int i, int i2, int i3);

    @Namespace("cv::aruco")
    @opencv_core.Ptr
    @Name({"generateCustomDictionary"})
    public static native Dictionary custom_dictionary_from(int i, int i2, @opencv_core.Ptr Dictionary dictionary);

    @Namespace("cv::aruco")
    @opencv_core.Ptr
    @Name({"generateCustomDictionary"})
    public static native Dictionary custom_dictionary_from(int i, int i2, @opencv_core.Ptr Dictionary dictionary, int i3);

    @Namespace("cv::aruco")
    public static native void detectCharucoDiamond(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat2, float f, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv::aruco")
    public static native void detectCharucoDiamond(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat2, float f, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat5);

    @Namespace("cv::aruco")
    public static native void detectCharucoDiamond(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat2, float f, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv::aruco")
    public static native void detectCharucoDiamond(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat2, float f, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat5);

    @Namespace("cv::aruco")
    public static native void detectCharucoDiamond(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat2, float f, @ByVal opencv_core.UMatVector uMatVector2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv::aruco")
    public static native void detectCharucoDiamond(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat2, float f, @ByVal opencv_core.UMatVector uMatVector2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat5);

    @Namespace("cv::aruco")
    public static native void detectCharucoDiamond(@ByVal opencv_core.Mat mat, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat2, float f, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv::aruco")
    public static native void detectCharucoDiamond(@ByVal opencv_core.Mat mat, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat2, float f, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByVal opencv_core.Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat5);

    @Namespace("cv::aruco")
    public static native void detectCharucoDiamond(@ByVal opencv_core.Mat mat, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat2, float f, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv::aruco")
    public static native void detectCharucoDiamond(@ByVal opencv_core.Mat mat, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat2, float f, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat5);

    @Namespace("cv::aruco")
    public static native void detectCharucoDiamond(@ByVal opencv_core.Mat mat, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat2, float f, @ByVal opencv_core.UMatVector uMatVector2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv::aruco")
    public static native void detectCharucoDiamond(@ByVal opencv_core.Mat mat, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat2, float f, @ByVal opencv_core.UMatVector uMatVector2, @ByVal opencv_core.Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat5);

    @Namespace("cv::aruco")
    public static native void detectCharucoDiamond(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat2, float f, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv::aruco")
    public static native void detectCharucoDiamond(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat2, float f, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByVal opencv_core.UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat5);

    @Namespace("cv::aruco")
    public static native void detectCharucoDiamond(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat2, float f, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv::aruco")
    public static native void detectCharucoDiamond(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat2, float f, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat5);

    @Namespace("cv::aruco")
    public static native void detectCharucoDiamond(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat2, float f, @ByVal opencv_core.UMatVector uMatVector2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv::aruco")
    public static native void detectCharucoDiamond(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat2, float f, @ByVal opencv_core.UMatVector uMatVector2, @ByVal opencv_core.UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat5);

    @Namespace("cv::aruco")
    public static native void detectMarkers(@ByVal opencv_core.GpuMat gpuMat, @opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::aruco")
    public static native void detectMarkers(@ByVal opencv_core.GpuMat gpuMat, @opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat2, @opencv_core.Ptr DetectorParameters detectorParameters, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat4);

    @Namespace("cv::aruco")
    public static native void detectMarkers(@ByVal opencv_core.GpuMat gpuMat, @opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::aruco")
    public static native void detectMarkers(@ByVal opencv_core.GpuMat gpuMat, @opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat2, @opencv_core.Ptr DetectorParameters detectorParameters, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat4);

    @Namespace("cv::aruco")
    public static native void detectMarkers(@ByVal opencv_core.GpuMat gpuMat, @opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::aruco")
    public static native void detectMarkers(@ByVal opencv_core.GpuMat gpuMat, @opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat2, @opencv_core.Ptr DetectorParameters detectorParameters, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat4);

    @Namespace("cv::aruco")
    public static native void detectMarkers(@ByVal opencv_core.Mat mat, @opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::aruco")
    public static native void detectMarkers(@ByVal opencv_core.Mat mat, @opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat2, @opencv_core.Ptr DetectorParameters detectorParameters, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat4);

    @Namespace("cv::aruco")
    public static native void detectMarkers(@ByVal opencv_core.Mat mat, @opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::aruco")
    public static native void detectMarkers(@ByVal opencv_core.Mat mat, @opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat2, @opencv_core.Ptr DetectorParameters detectorParameters, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat4);

    @Namespace("cv::aruco")
    public static native void detectMarkers(@ByVal opencv_core.Mat mat, @opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::aruco")
    public static native void detectMarkers(@ByVal opencv_core.Mat mat, @opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat2, @opencv_core.Ptr DetectorParameters detectorParameters, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat4);

    @Namespace("cv::aruco")
    public static native void detectMarkers(@ByVal opencv_core.UMat uMat, @opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::aruco")
    public static native void detectMarkers(@ByVal opencv_core.UMat uMat, @opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat2, @opencv_core.Ptr DetectorParameters detectorParameters, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat4);

    @Namespace("cv::aruco")
    public static native void detectMarkers(@ByVal opencv_core.UMat uMat, @opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::aruco")
    public static native void detectMarkers(@ByVal opencv_core.UMat uMat, @opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat2, @opencv_core.Ptr DetectorParameters detectorParameters, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat4);

    @Namespace("cv::aruco")
    public static native void detectMarkers(@ByVal opencv_core.UMat uMat, @opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::aruco")
    public static native void detectMarkers(@ByVal opencv_core.UMat uMat, @opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat2, @opencv_core.Ptr DetectorParameters detectorParameters, @ByVal(nullValue = "cv::OutputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat4);

    @Namespace("cv::aruco")
    public static native void drawAxis(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, float f);

    @Namespace("cv::aruco")
    public static native void drawAxis(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, float f);

    @Namespace("cv::aruco")
    public static native void drawAxis(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, float f);

    @Namespace("cv::aruco")
    public static native void drawCharucoDiamond(@opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.Scalar4i scalar4i, int i, int i2, @ByVal opencv_core.GpuMat gpuMat);

    @Namespace("cv::aruco")
    public static native void drawCharucoDiamond(@opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.Scalar4i scalar4i, int i, int i2, @ByVal opencv_core.GpuMat gpuMat, int i3, int i4);

    @Namespace("cv::aruco")
    public static native void drawCharucoDiamond(@opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.Scalar4i scalar4i, int i, int i2, @ByVal opencv_core.Mat mat);

    @Namespace("cv::aruco")
    public static native void drawCharucoDiamond(@opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.Scalar4i scalar4i, int i, int i2, @ByVal opencv_core.Mat mat, int i3, int i4);

    @Namespace("cv::aruco")
    public static native void drawCharucoDiamond(@opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.Scalar4i scalar4i, int i, int i2, @ByVal opencv_core.UMat uMat);

    @Namespace("cv::aruco")
    public static native void drawCharucoDiamond(@opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.Scalar4i scalar4i, int i, int i2, @ByVal opencv_core.UMat uMat, int i3, int i4);

    @Namespace("cv::aruco")
    public static native void drawDetectedCornersCharuco(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::aruco")
    public static native void drawDetectedCornersCharuco(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat3, @ByVal(nullValue = "cv::Scalar(255, 0, 0)") opencv_core.Scalar scalar);

    @Namespace("cv::aruco")
    public static native void drawDetectedCornersCharuco(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::aruco")
    public static native void drawDetectedCornersCharuco(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat3, @ByVal(nullValue = "cv::Scalar(255, 0, 0)") opencv_core.Scalar scalar);

    @Namespace("cv::aruco")
    public static native void drawDetectedCornersCharuco(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::aruco")
    public static native void drawDetectedCornersCharuco(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat3, @ByVal(nullValue = "cv::Scalar(255, 0, 0)") opencv_core.Scalar scalar);

    @Namespace("cv::aruco")
    public static native void drawDetectedDiamonds(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMatVector gpuMatVector);

    @Namespace("cv::aruco")
    public static native void drawDetectedDiamonds(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat2, @ByVal(nullValue = "cv::Scalar(0, 0, 255)") opencv_core.Scalar scalar);

    @Namespace("cv::aruco")
    public static native void drawDetectedDiamonds(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.MatVector matVector);

    @Namespace("cv::aruco")
    public static native void drawDetectedDiamonds(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.MatVector matVector, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat2, @ByVal(nullValue = "cv::Scalar(0, 0, 255)") opencv_core.Scalar scalar);

    @Namespace("cv::aruco")
    public static native void drawDetectedDiamonds(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.UMatVector uMatVector);

    @Namespace("cv::aruco")
    public static native void drawDetectedDiamonds(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.UMatVector uMatVector, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat2, @ByVal(nullValue = "cv::Scalar(0, 0, 255)") opencv_core.Scalar scalar);

    @Namespace("cv::aruco")
    public static native void drawDetectedDiamonds(@ByVal opencv_core.Mat mat, @ByVal opencv_core.GpuMatVector gpuMatVector);

    @Namespace("cv::aruco")
    public static native void drawDetectedDiamonds(@ByVal opencv_core.Mat mat, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat2, @ByVal(nullValue = "cv::Scalar(0, 0, 255)") opencv_core.Scalar scalar);

    @Namespace("cv::aruco")
    public static native void drawDetectedDiamonds(@ByVal opencv_core.Mat mat, @ByVal opencv_core.MatVector matVector);

    @Namespace("cv::aruco")
    public static native void drawDetectedDiamonds(@ByVal opencv_core.Mat mat, @ByVal opencv_core.MatVector matVector, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat2, @ByVal(nullValue = "cv::Scalar(0, 0, 255)") opencv_core.Scalar scalar);

    @Namespace("cv::aruco")
    public static native void drawDetectedDiamonds(@ByVal opencv_core.Mat mat, @ByVal opencv_core.UMatVector uMatVector);

    @Namespace("cv::aruco")
    public static native void drawDetectedDiamonds(@ByVal opencv_core.Mat mat, @ByVal opencv_core.UMatVector uMatVector, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat2, @ByVal(nullValue = "cv::Scalar(0, 0, 255)") opencv_core.Scalar scalar);

    @Namespace("cv::aruco")
    public static native void drawDetectedDiamonds(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.GpuMatVector gpuMatVector);

    @Namespace("cv::aruco")
    public static native void drawDetectedDiamonds(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat2, @ByVal(nullValue = "cv::Scalar(0, 0, 255)") opencv_core.Scalar scalar);

    @Namespace("cv::aruco")
    public static native void drawDetectedDiamonds(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.MatVector matVector);

    @Namespace("cv::aruco")
    public static native void drawDetectedDiamonds(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.MatVector matVector, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat2, @ByVal(nullValue = "cv::Scalar(0, 0, 255)") opencv_core.Scalar scalar);

    @Namespace("cv::aruco")
    public static native void drawDetectedDiamonds(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMatVector uMatVector);

    @Namespace("cv::aruco")
    public static native void drawDetectedDiamonds(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMatVector uMatVector, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat2, @ByVal(nullValue = "cv::Scalar(0, 0, 255)") opencv_core.Scalar scalar);

    @Namespace("cv::aruco")
    public static native void drawDetectedMarkers(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMatVector gpuMatVector);

    @Namespace("cv::aruco")
    public static native void drawDetectedMarkers(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat2, @ByVal(nullValue = "cv::Scalar(0, 255, 0)") opencv_core.Scalar scalar);

    @Namespace("cv::aruco")
    public static native void drawDetectedMarkers(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.MatVector matVector);

    @Namespace("cv::aruco")
    public static native void drawDetectedMarkers(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.MatVector matVector, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat2, @ByVal(nullValue = "cv::Scalar(0, 255, 0)") opencv_core.Scalar scalar);

    @Namespace("cv::aruco")
    public static native void drawDetectedMarkers(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.UMatVector uMatVector);

    @Namespace("cv::aruco")
    public static native void drawDetectedMarkers(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.UMatVector uMatVector, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat2, @ByVal(nullValue = "cv::Scalar(0, 255, 0)") opencv_core.Scalar scalar);

    @Namespace("cv::aruco")
    public static native void drawDetectedMarkers(@ByVal opencv_core.Mat mat, @ByVal opencv_core.GpuMatVector gpuMatVector);

    @Namespace("cv::aruco")
    public static native void drawDetectedMarkers(@ByVal opencv_core.Mat mat, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat2, @ByVal(nullValue = "cv::Scalar(0, 255, 0)") opencv_core.Scalar scalar);

    @Namespace("cv::aruco")
    public static native void drawDetectedMarkers(@ByVal opencv_core.Mat mat, @ByVal opencv_core.MatVector matVector);

    @Namespace("cv::aruco")
    public static native void drawDetectedMarkers(@ByVal opencv_core.Mat mat, @ByVal opencv_core.MatVector matVector, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat2, @ByVal(nullValue = "cv::Scalar(0, 255, 0)") opencv_core.Scalar scalar);

    @Namespace("cv::aruco")
    public static native void drawDetectedMarkers(@ByVal opencv_core.Mat mat, @ByVal opencv_core.UMatVector uMatVector);

    @Namespace("cv::aruco")
    public static native void drawDetectedMarkers(@ByVal opencv_core.Mat mat, @ByVal opencv_core.UMatVector uMatVector, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat2, @ByVal(nullValue = "cv::Scalar(0, 255, 0)") opencv_core.Scalar scalar);

    @Namespace("cv::aruco")
    public static native void drawDetectedMarkers(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.GpuMatVector gpuMatVector);

    @Namespace("cv::aruco")
    public static native void drawDetectedMarkers(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat2, @ByVal(nullValue = "cv::Scalar(0, 255, 0)") opencv_core.Scalar scalar);

    @Namespace("cv::aruco")
    public static native void drawDetectedMarkers(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.MatVector matVector);

    @Namespace("cv::aruco")
    public static native void drawDetectedMarkers(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.MatVector matVector, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat2, @ByVal(nullValue = "cv::Scalar(0, 255, 0)") opencv_core.Scalar scalar);

    @Namespace("cv::aruco")
    public static native void drawDetectedMarkers(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMatVector uMatVector);

    @Namespace("cv::aruco")
    public static native void drawDetectedMarkers(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMatVector uMatVector, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat2, @ByVal(nullValue = "cv::Scalar(0, 255, 0)") opencv_core.Scalar scalar);

    @Namespace("cv::aruco")
    public static native void drawMarker(@opencv_core.Ptr Dictionary dictionary, int i, int i2, @ByVal opencv_core.GpuMat gpuMat);

    @Namespace("cv::aruco")
    public static native void drawMarker(@opencv_core.Ptr Dictionary dictionary, int i, int i2, @ByVal opencv_core.GpuMat gpuMat, int i3);

    @Namespace("cv::aruco")
    public static native void drawMarker(@opencv_core.Ptr Dictionary dictionary, int i, int i2, @ByVal opencv_core.Mat mat);

    @Namespace("cv::aruco")
    public static native void drawMarker(@opencv_core.Ptr Dictionary dictionary, int i, int i2, @ByVal opencv_core.Mat mat, int i3);

    @Namespace("cv::aruco")
    public static native void drawMarker(@opencv_core.Ptr Dictionary dictionary, int i, int i2, @ByVal opencv_core.UMat uMat);

    @Namespace("cv::aruco")
    public static native void drawMarker(@opencv_core.Ptr Dictionary dictionary, int i, int i2, @ByVal opencv_core.UMat uMat, int i3);

    @Namespace("cv::aruco")
    public static native void drawPlanarBoard(@opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat);

    @Namespace("cv::aruco")
    public static native void drawPlanarBoard(@opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat, int i, int i2);

    @Namespace("cv::aruco")
    public static native void drawPlanarBoard(@opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat);

    @Namespace("cv::aruco")
    public static native void drawPlanarBoard(@opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat, int i, int i2);

    @Namespace("cv::aruco")
    public static native void drawPlanarBoard(@opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat);

    @Namespace("cv::aruco")
    public static native void drawPlanarBoard(@opencv_core.Ptr Board board, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat, int i, int i2);

    @Namespace("cv::aruco")
    public static native int estimatePoseBoard(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat, @opencv_core.Ptr Board board, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5);

    @Namespace("cv::aruco")
    public static native int estimatePoseBoard(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat, @opencv_core.Ptr Board board, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, @Cast({"bool"}) boolean z);

    @Namespace("cv::aruco")
    public static native int estimatePoseBoard(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat, @opencv_core.Ptr Board board, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5);

    @Namespace("cv::aruco")
    public static native int estimatePoseBoard(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat, @opencv_core.Ptr Board board, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @Cast({"bool"}) boolean z);

    @Namespace("cv::aruco")
    public static native int estimatePoseBoard(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat, @opencv_core.Ptr Board board, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5);

    @Namespace("cv::aruco")
    public static native int estimatePoseBoard(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat, @opencv_core.Ptr Board board, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, @Cast({"bool"}) boolean z);

    @Namespace("cv::aruco")
    public static native int estimatePoseBoard(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, @opencv_core.Ptr Board board, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5);

    @Namespace("cv::aruco")
    public static native int estimatePoseBoard(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, @opencv_core.Ptr Board board, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, @Cast({"bool"}) boolean z);

    @Namespace("cv::aruco")
    public static native int estimatePoseBoard(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, @opencv_core.Ptr Board board, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5);

    @Namespace("cv::aruco")
    public static native int estimatePoseBoard(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, @opencv_core.Ptr Board board, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @Cast({"bool"}) boolean z);

    @Namespace("cv::aruco")
    public static native int estimatePoseBoard(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, @opencv_core.Ptr Board board, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5);

    @Namespace("cv::aruco")
    public static native int estimatePoseBoard(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, @opencv_core.Ptr Board board, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, @Cast({"bool"}) boolean z);

    @Namespace("cv::aruco")
    public static native int estimatePoseBoard(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat, @opencv_core.Ptr Board board, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5);

    @Namespace("cv::aruco")
    public static native int estimatePoseBoard(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat, @opencv_core.Ptr Board board, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, @Cast({"bool"}) boolean z);

    @Namespace("cv::aruco")
    public static native int estimatePoseBoard(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat, @opencv_core.Ptr Board board, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5);

    @Namespace("cv::aruco")
    public static native int estimatePoseBoard(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat, @opencv_core.Ptr Board board, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @Cast({"bool"}) boolean z);

    @Namespace("cv::aruco")
    public static native int estimatePoseBoard(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat, @opencv_core.Ptr Board board, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5);

    @Namespace("cv::aruco")
    public static native int estimatePoseBoard(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat, @opencv_core.Ptr Board board, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, @Cast({"bool"}) boolean z);

    @Namespace("cv::aruco")
    @Cast({"bool"})
    public static native boolean estimatePoseCharucoBoard(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6);

    @Namespace("cv::aruco")
    @Cast({"bool"})
    public static native boolean estimatePoseCharucoBoard(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6, @Cast({"bool"}) boolean z);

    @Namespace("cv::aruco")
    @Cast({"bool"})
    public static native boolean estimatePoseCharucoBoard(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6);

    @Namespace("cv::aruco")
    @Cast({"bool"})
    public static native boolean estimatePoseCharucoBoard(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, @Cast({"bool"}) boolean z);

    @Namespace("cv::aruco")
    @Cast({"bool"})
    public static native boolean estimatePoseCharucoBoard(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6);

    @Namespace("cv::aruco")
    @Cast({"bool"})
    public static native boolean estimatePoseCharucoBoard(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6, @Cast({"bool"}) boolean z);

    @Namespace("cv::aruco")
    public static native void estimatePoseSingleMarkers(@ByVal opencv_core.GpuMatVector gpuMatVector, float f, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4);

    @Namespace("cv::aruco")
    public static native void estimatePoseSingleMarkers(@ByVal opencv_core.GpuMatVector gpuMatVector, float f, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat5);

    @Namespace("cv::aruco")
    public static native void estimatePoseSingleMarkers(@ByVal opencv_core.GpuMatVector gpuMatVector, float f, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4);

    @Namespace("cv::aruco")
    public static native void estimatePoseSingleMarkers(@ByVal opencv_core.GpuMatVector gpuMatVector, float f, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat5);

    @Namespace("cv::aruco")
    public static native void estimatePoseSingleMarkers(@ByVal opencv_core.GpuMatVector gpuMatVector, float f, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4);

    @Namespace("cv::aruco")
    public static native void estimatePoseSingleMarkers(@ByVal opencv_core.GpuMatVector gpuMatVector, float f, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat5);

    @Namespace("cv::aruco")
    public static native void estimatePoseSingleMarkers(@ByVal opencv_core.MatVector matVector, float f, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4);

    @Namespace("cv::aruco")
    public static native void estimatePoseSingleMarkers(@ByVal opencv_core.MatVector matVector, float f, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat5);

    @Namespace("cv::aruco")
    public static native void estimatePoseSingleMarkers(@ByVal opencv_core.MatVector matVector, float f, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4);

    @Namespace("cv::aruco")
    public static native void estimatePoseSingleMarkers(@ByVal opencv_core.MatVector matVector, float f, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat5);

    @Namespace("cv::aruco")
    public static native void estimatePoseSingleMarkers(@ByVal opencv_core.MatVector matVector, float f, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4);

    @Namespace("cv::aruco")
    public static native void estimatePoseSingleMarkers(@ByVal opencv_core.MatVector matVector, float f, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat5);

    @Namespace("cv::aruco")
    public static native void estimatePoseSingleMarkers(@ByVal opencv_core.UMatVector uMatVector, float f, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4);

    @Namespace("cv::aruco")
    public static native void estimatePoseSingleMarkers(@ByVal opencv_core.UMatVector uMatVector, float f, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat5);

    @Namespace("cv::aruco")
    public static native void estimatePoseSingleMarkers(@ByVal opencv_core.UMatVector uMatVector, float f, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4);

    @Namespace("cv::aruco")
    public static native void estimatePoseSingleMarkers(@ByVal opencv_core.UMatVector uMatVector, float f, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat5);

    @Namespace("cv::aruco")
    public static native void estimatePoseSingleMarkers(@ByVal opencv_core.UMatVector uMatVector, float f, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4);

    @Namespace("cv::aruco")
    public static native void estimatePoseSingleMarkers(@ByVal opencv_core.UMatVector uMatVector, float f, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat5);

    @Namespace("cv::aruco")
    public static native void getBoardObjectAndImagePoints(@opencv_core.Ptr Board board, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv::aruco")
    public static native void getBoardObjectAndImagePoints(@opencv_core.Ptr Board board, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv::aruco")
    public static native void getBoardObjectAndImagePoints(@opencv_core.Ptr Board board, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv::aruco")
    public static native void getBoardObjectAndImagePoints(@opencv_core.Ptr Board board, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv::aruco")
    public static native void getBoardObjectAndImagePoints(@opencv_core.Ptr Board board, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv::aruco")
    public static native void getBoardObjectAndImagePoints(@opencv_core.Ptr Board board, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv::aruco")
    public static native void getBoardObjectAndImagePoints(@opencv_core.Ptr Board board, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv::aruco")
    public static native void getBoardObjectAndImagePoints(@opencv_core.Ptr Board board, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv::aruco")
    public static native void getBoardObjectAndImagePoints(@opencv_core.Ptr Board board, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv::aruco")
    @opencv_core.Ptr
    public static native Dictionary getPredefinedDictionary(@Cast({"cv::aruco::PREDEFINED_DICTIONARY_NAME"}) int i);

    @Namespace("cv::aruco")
    public static native int interpolateCornersCharuco(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4);

    @Namespace("cv::aruco")
    public static native int interpolateCornersCharuco(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat5, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat6, int i);

    @Namespace("cv::aruco")
    public static native int interpolateCornersCharuco(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4);

    @Namespace("cv::aruco")
    public static native int interpolateCornersCharuco(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat5, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat6, int i);

    @Namespace("cv::aruco")
    public static native int interpolateCornersCharuco(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4);

    @Namespace("cv::aruco")
    public static native int interpolateCornersCharuco(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat5, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat6, int i);

    @Namespace("cv::aruco")
    public static native int interpolateCornersCharuco(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4);

    @Namespace("cv::aruco")
    public static native int interpolateCornersCharuco(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat5, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat6, int i);

    @Namespace("cv::aruco")
    public static native int interpolateCornersCharuco(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4);

    @Namespace("cv::aruco")
    public static native int interpolateCornersCharuco(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat5, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat6, int i);

    @Namespace("cv::aruco")
    public static native int interpolateCornersCharuco(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4);

    @Namespace("cv::aruco")
    public static native int interpolateCornersCharuco(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat5, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat6, int i);

    @Namespace("cv::aruco")
    public static native int interpolateCornersCharuco(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4);

    @Namespace("cv::aruco")
    public static native int interpolateCornersCharuco(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat5, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat6, int i);

    @Namespace("cv::aruco")
    public static native int interpolateCornersCharuco(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4);

    @Namespace("cv::aruco")
    public static native int interpolateCornersCharuco(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat5, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat6, int i);

    @Namespace("cv::aruco")
    public static native int interpolateCornersCharuco(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4);

    @Namespace("cv::aruco")
    public static native int interpolateCornersCharuco(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @opencv_core.Ptr CharucoBoard charucoBoard, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat5, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat6, int i);

    @Namespace("cv::aruco")
    public static native void refineDetectedMarkers(@ByVal opencv_core.GpuMat gpuMat, @opencv_core.Ptr Board board, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMatVector gpuMatVector2);

    @Namespace("cv::aruco")
    public static native void refineDetectedMarkers(@ByVal opencv_core.GpuMat gpuMat, @opencv_core.Ptr Board board, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat4, float f, float f2, @Cast({"bool"}) boolean z, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat5, @opencv_core.Ptr DetectorParameters detectorParameters);

    @Namespace("cv::aruco")
    public static native void refineDetectedMarkers(@ByVal opencv_core.GpuMat gpuMat, @opencv_core.Ptr Board board, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.MatVector matVector2);

    @Namespace("cv::aruco")
    public static native void refineDetectedMarkers(@ByVal opencv_core.GpuMat gpuMat, @opencv_core.Ptr Board board, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.MatVector matVector2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat4, float f, float f2, @Cast({"bool"}) boolean z, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat5, @opencv_core.Ptr DetectorParameters detectorParameters);

    @Namespace("cv::aruco")
    public static native void refineDetectedMarkers(@ByVal opencv_core.GpuMat gpuMat, @opencv_core.Ptr Board board, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.UMatVector uMatVector2);

    @Namespace("cv::aruco")
    public static native void refineDetectedMarkers(@ByVal opencv_core.GpuMat gpuMat, @opencv_core.Ptr Board board, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.UMatVector uMatVector2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat4, float f, float f2, @Cast({"bool"}) boolean z, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat5, @opencv_core.Ptr DetectorParameters detectorParameters);

    @Namespace("cv::aruco")
    public static native void refineDetectedMarkers(@ByVal opencv_core.Mat mat, @opencv_core.Ptr Board board, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.GpuMatVector gpuMatVector2);

    @Namespace("cv::aruco")
    public static native void refineDetectedMarkers(@ByVal opencv_core.Mat mat, @opencv_core.Ptr Board board, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat4, float f, float f2, @Cast({"bool"}) boolean z, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat5, @opencv_core.Ptr DetectorParameters detectorParameters);

    @Namespace("cv::aruco")
    public static native void refineDetectedMarkers(@ByVal opencv_core.Mat mat, @opencv_core.Ptr Board board, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.MatVector matVector2);

    @Namespace("cv::aruco")
    public static native void refineDetectedMarkers(@ByVal opencv_core.Mat mat, @opencv_core.Ptr Board board, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.MatVector matVector2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat4, float f, float f2, @Cast({"bool"}) boolean z, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat5, @opencv_core.Ptr DetectorParameters detectorParameters);

    @Namespace("cv::aruco")
    public static native void refineDetectedMarkers(@ByVal opencv_core.Mat mat, @opencv_core.Ptr Board board, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.UMatVector uMatVector2);

    @Namespace("cv::aruco")
    public static native void refineDetectedMarkers(@ByVal opencv_core.Mat mat, @opencv_core.Ptr Board board, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.UMatVector uMatVector2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat4, float f, float f2, @Cast({"bool"}) boolean z, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat5, @opencv_core.Ptr DetectorParameters detectorParameters);

    @Namespace("cv::aruco")
    public static native void refineDetectedMarkers(@ByVal opencv_core.UMat uMat, @opencv_core.Ptr Board board, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.GpuMatVector gpuMatVector2);

    @Namespace("cv::aruco")
    public static native void refineDetectedMarkers(@ByVal opencv_core.UMat uMat, @opencv_core.Ptr Board board, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat4, float f, float f2, @Cast({"bool"}) boolean z, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat5, @opencv_core.Ptr DetectorParameters detectorParameters);

    @Namespace("cv::aruco")
    public static native void refineDetectedMarkers(@ByVal opencv_core.UMat uMat, @opencv_core.Ptr Board board, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.MatVector matVector2);

    @Namespace("cv::aruco")
    public static native void refineDetectedMarkers(@ByVal opencv_core.UMat uMat, @opencv_core.Ptr Board board, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.MatVector matVector2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat4, float f, float f2, @Cast({"bool"}) boolean z, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat5, @opencv_core.Ptr DetectorParameters detectorParameters);

    @Namespace("cv::aruco")
    public static native void refineDetectedMarkers(@ByVal opencv_core.UMat uMat, @opencv_core.Ptr Board board, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMatVector uMatVector2);

    @Namespace("cv::aruco")
    public static native void refineDetectedMarkers(@ByVal opencv_core.UMat uMat, @opencv_core.Ptr Board board, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMatVector uMatVector2, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat4, float f, float f2, @Cast({"bool"}) boolean z, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat5, @opencv_core.Ptr DetectorParameters detectorParameters);

    static {
        Loader.load();
    }

    @Namespace("cv::aruco")
    @NoOffset
    public static class Dictionary extends Pointer {
        private native void allocate();

        private native void allocate(@opencv_core.Ptr Dictionary dictionary);

        private native void allocate(@ByRef(nullValue = "cv::Mat()") @Const opencv_core.Mat mat, int i, int i2);

        private native void allocateArray(long j);

        @opencv_core.Ptr
        public static native Dictionary create(int i, int i2);

        @opencv_core.Ptr
        public static native Dictionary create(int i, int i2, int i3);

        @opencv_core.Ptr
        @Name({"create"})
        public static native Dictionary create_from(int i, int i2, @opencv_core.Ptr Dictionary dictionary);

        @opencv_core.Ptr
        @Name({"create"})
        public static native Dictionary create_from(int i, int i2, @opencv_core.Ptr Dictionary dictionary, int i3);

        @opencv_core.Ptr
        public static native Dictionary get(int i);

        @ByVal
        public static native opencv_core.Mat getBitsFromByteList(@ByRef @Const opencv_core.Mat mat, int i);

        @ByVal
        public static native opencv_core.Mat getByteListFromBits(@ByRef @Const opencv_core.Mat mat);

        public native Dictionary bytesList(opencv_core.Mat mat);

        @ByRef
        public native opencv_core.Mat bytesList();

        public native void drawMarker(int i, int i2, @ByVal opencv_core.GpuMat gpuMat);

        public native void drawMarker(int i, int i2, @ByVal opencv_core.GpuMat gpuMat, int i3);

        public native void drawMarker(int i, int i2, @ByVal opencv_core.Mat mat);

        public native void drawMarker(int i, int i2, @ByVal opencv_core.Mat mat, int i3);

        public native void drawMarker(int i, int i2, @ByVal opencv_core.UMat uMat);

        public native void drawMarker(int i, int i2, @ByVal opencv_core.UMat uMat, int i3);

        public native int getDistanceToId(@ByVal opencv_core.GpuMat gpuMat, int i);

        public native int getDistanceToId(@ByVal opencv_core.GpuMat gpuMat, int i, @Cast({"bool"}) boolean z);

        public native int getDistanceToId(@ByVal opencv_core.Mat mat, int i);

        public native int getDistanceToId(@ByVal opencv_core.Mat mat, int i, @Cast({"bool"}) boolean z);

        public native int getDistanceToId(@ByVal opencv_core.UMat uMat, int i);

        public native int getDistanceToId(@ByVal opencv_core.UMat uMat, int i, @Cast({"bool"}) boolean z);

        @Cast({"bool"})
        public native boolean identify(@ByRef @Const opencv_core.Mat mat, @ByRef IntBuffer intBuffer, @ByRef IntBuffer intBuffer2, double d);

        @Cast({"bool"})
        public native boolean identify(@ByRef @Const opencv_core.Mat mat, @ByRef IntPointer intPointer, @ByRef IntPointer intPointer2, double d);

        @Cast({"bool"})
        public native boolean identify(@ByRef @Const opencv_core.Mat mat, @ByRef int[] iArr, @ByRef int[] iArr2, double d);

        public native int markerSize();

        public native Dictionary markerSize(int i);

        public native int maxCorrectionBits();

        public native Dictionary maxCorrectionBits(int i);

        static {
            Loader.load();
        }

        public Dictionary(Pointer p) {
            super(p);
        }

        public Dictionary(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Dictionary position(long position) {
            return (Dictionary) super.position(position);
        }

        public Dictionary(@ByRef(nullValue = "cv::Mat()") @Const opencv_core.Mat _bytesList, int _markerSize, int _maxcorr) {
            super((Pointer) null);
            allocate(_bytesList, _markerSize, _maxcorr);
        }

        public Dictionary() {
            super((Pointer) null);
            allocate();
        }

        public Dictionary(@opencv_core.Ptr Dictionary _dictionary) {
            super((Pointer) null);
            allocate(_dictionary);
        }
    }

    @Namespace("cv::aruco")
    @NoOffset
    public static class DetectorParameters extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        @opencv_core.Ptr
        public static native DetectorParameters create();

        public native double adaptiveThreshConstant();

        public native DetectorParameters adaptiveThreshConstant(double d);

        public native int adaptiveThreshWinSizeMax();

        public native DetectorParameters adaptiveThreshWinSizeMax(int i);

        public native int adaptiveThreshWinSizeMin();

        public native DetectorParameters adaptiveThreshWinSizeMin(int i);

        public native int adaptiveThreshWinSizeStep();

        public native DetectorParameters adaptiveThreshWinSizeStep(int i);

        public native float aprilTagCriticalRad();

        public native DetectorParameters aprilTagCriticalRad(float f);

        public native int aprilTagDeglitch();

        public native DetectorParameters aprilTagDeglitch(int i);

        public native float aprilTagMaxLineFitMse();

        public native DetectorParameters aprilTagMaxLineFitMse(float f);

        public native int aprilTagMaxNmaxima();

        public native DetectorParameters aprilTagMaxNmaxima(int i);

        public native int aprilTagMinClusterPixels();

        public native DetectorParameters aprilTagMinClusterPixels(int i);

        public native int aprilTagMinWhiteBlackDiff();

        public native DetectorParameters aprilTagMinWhiteBlackDiff(int i);

        public native float aprilTagQuadDecimate();

        public native DetectorParameters aprilTagQuadDecimate(float f);

        public native float aprilTagQuadSigma();

        public native DetectorParameters aprilTagQuadSigma(float f);

        public native int cornerRefinementMaxIterations();

        public native DetectorParameters cornerRefinementMaxIterations(int i);

        public native int cornerRefinementMethod();

        public native DetectorParameters cornerRefinementMethod(int i);

        public native double cornerRefinementMinAccuracy();

        public native DetectorParameters cornerRefinementMinAccuracy(double d);

        public native int cornerRefinementWinSize();

        public native DetectorParameters cornerRefinementWinSize(int i);

        public native double errorCorrectionRate();

        public native DetectorParameters errorCorrectionRate(double d);

        public native int markerBorderBits();

        public native DetectorParameters markerBorderBits(int i);

        public native double maxErroneousBitsInBorderRate();

        public native DetectorParameters maxErroneousBitsInBorderRate(double d);

        public native double maxMarkerPerimeterRate();

        public native DetectorParameters maxMarkerPerimeterRate(double d);

        public native double minCornerDistanceRate();

        public native DetectorParameters minCornerDistanceRate(double d);

        public native int minDistanceToBorder();

        public native DetectorParameters minDistanceToBorder(int i);

        public native double minMarkerDistanceRate();

        public native DetectorParameters minMarkerDistanceRate(double d);

        public native double minMarkerPerimeterRate();

        public native DetectorParameters minMarkerPerimeterRate(double d);

        public native double minOtsuStdDev();

        public native DetectorParameters minOtsuStdDev(double d);

        public native double perspectiveRemoveIgnoredMarginPerCell();

        public native DetectorParameters perspectiveRemoveIgnoredMarginPerCell(double d);

        public native int perspectiveRemovePixelPerCell();

        public native DetectorParameters perspectiveRemovePixelPerCell(int i);

        public native double polygonalApproxAccuracyRate();

        public native DetectorParameters polygonalApproxAccuracyRate(double d);

        static {
            Loader.load();
        }

        public DetectorParameters(Pointer p) {
            super(p);
        }

        public DetectorParameters(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public DetectorParameters position(long position) {
            return (DetectorParameters) super.position(position);
        }

        public DetectorParameters() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::aruco")
    public static class Board extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        @opencv_core.Ptr
        public static native Board create(@ByVal opencv_core.GpuMatVector gpuMatVector, @opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.GpuMat gpuMat);

        @opencv_core.Ptr
        public static native Board create(@ByVal opencv_core.GpuMatVector gpuMatVector, @opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.Mat mat);

        @opencv_core.Ptr
        public static native Board create(@ByVal opencv_core.GpuMatVector gpuMatVector, @opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.UMat uMat);

        @opencv_core.Ptr
        public static native Board create(@ByVal opencv_core.MatVector matVector, @opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.GpuMat gpuMat);

        @opencv_core.Ptr
        public static native Board create(@ByVal opencv_core.MatVector matVector, @opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.Mat mat);

        @opencv_core.Ptr
        public static native Board create(@ByVal opencv_core.MatVector matVector, @opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.UMat uMat);

        @opencv_core.Ptr
        public static native Board create(@ByVal opencv_core.UMatVector uMatVector, @opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.GpuMat gpuMat);

        @opencv_core.Ptr
        public static native Board create(@ByVal opencv_core.UMatVector uMatVector, @opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.Mat mat);

        @opencv_core.Ptr
        public static native Board create(@ByVal opencv_core.UMatVector uMatVector, @opencv_core.Ptr Dictionary dictionary, @ByVal opencv_core.UMat uMat);

        public native Board dictionary(Dictionary dictionary);

        @opencv_core.Ptr
        public native Dictionary dictionary();

        @StdVector
        public native IntPointer ids();

        public native Board ids(IntPointer intPointer);

        public native Board objPoints(opencv_core.Point3fVectorVector point3fVectorVector);

        @ByRef
        public native opencv_core.Point3fVectorVector objPoints();

        static {
            Loader.load();
        }

        public Board() {
            super((Pointer) null);
            allocate();
        }

        public Board(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Board(Pointer p) {
            super(p);
        }

        public Board position(long position) {
            return (Board) super.position(position);
        }
    }

    @Namespace("cv::aruco")
    @NoOffset
    public static class GridBoard extends Board {
        private native void allocate();

        private native void allocateArray(long j);

        @opencv_core.Ptr
        public static native GridBoard create(int i, int i2, float f, float f2, @opencv_core.Ptr Dictionary dictionary);

        @opencv_core.Ptr
        public static native GridBoard create(int i, int i2, float f, float f2, @opencv_core.Ptr Dictionary dictionary, int i3);

        public native void draw(@ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat);

        public native void draw(@ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat, int i, int i2);

        public native void draw(@ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat);

        public native void draw(@ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat, int i, int i2);

        public native void draw(@ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat);

        public native void draw(@ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat, int i, int i2);

        @ByVal
        public native opencv_core.Size getGridSize();

        public native float getMarkerLength();

        public native float getMarkerSeparation();

        static {
            Loader.load();
        }

        public GridBoard() {
            super((Pointer) null);
            allocate();
        }

        public GridBoard(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public GridBoard(Pointer p) {
            super(p);
        }

        public GridBoard position(long position) {
            return (GridBoard) super.position(position);
        }
    }

    @Namespace("cv::aruco")
    @NoOffset
    public static class CharucoBoard extends Board {
        private native void allocate();

        private native void allocateArray(long j);

        @opencv_core.Ptr
        public static native CharucoBoard create(int i, int i2, float f, float f2, @opencv_core.Ptr Dictionary dictionary);

        public native CharucoBoard chessboardCorners(opencv_core.Point3fVector point3fVector);

        @ByRef
        @Cast({"std::vector<cv::Point3f>*"})
        public native opencv_core.Point3fVector chessboardCorners();

        public native void draw(@ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat);

        public native void draw(@ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat, int i, int i2);

        public native void draw(@ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat);

        public native void draw(@ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat, int i, int i2);

        public native void draw(@ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat);

        public native void draw(@ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat, int i, int i2);

        @ByVal
        public native opencv_core.Size getChessboardSize();

        public native float getMarkerLength();

        public native float getSquareLength();

        public native CharucoBoard nearestMarkerCorners(opencv_core.IntVectorVector intVectorVector);

        @ByRef
        public native opencv_core.IntVectorVector nearestMarkerCorners();

        public native CharucoBoard nearestMarkerIdx(opencv_core.IntVectorVector intVectorVector);

        @ByRef
        public native opencv_core.IntVectorVector nearestMarkerIdx();

        static {
            Loader.load();
        }

        public CharucoBoard() {
            super((Pointer) null);
            allocate();
        }

        public CharucoBoard(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CharucoBoard(Pointer p) {
            super(p);
        }

        public CharucoBoard position(long position) {
            return (CharucoBoard) super.position(position);
        }
    }
}
