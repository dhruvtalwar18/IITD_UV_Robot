package org.bytedeco.javacpp;

import java.nio.DoubleBuffer;
import org.bytedeco.javacpp.annotation.ByPtrRef;
import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Const;
import org.bytedeco.javacpp.annotation.Name;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.annotation.NoOffset;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.opencv_features2d;
import org.bytedeco.javacpp.presets.opencv_core;

public class opencv_calib3d extends org.bytedeco.javacpp.helper.opencv_calib3d {
    public static final int CALIB_CB_ACCURACY = 32;
    public static final int CALIB_CB_ADAPTIVE_THRESH = 1;
    public static final int CALIB_CB_ASYMMETRIC_GRID = 2;
    public static final int CALIB_CB_CLUSTERING = 4;
    public static final int CALIB_CB_EXHAUSTIVE = 16;
    public static final int CALIB_CB_FAST_CHECK = 8;
    public static final int CALIB_CB_FILTER_QUADS = 4;
    public static final int CALIB_CB_NORMALIZE_IMAGE = 2;
    public static final int CALIB_CB_SYMMETRIC_GRID = 1;
    public static final int CALIB_FIX_ASPECT_RATIO = 2;
    public static final int CALIB_FIX_FOCAL_LENGTH = 16;
    public static final int CALIB_FIX_INTRINSIC = 256;
    public static final int CALIB_FIX_K1 = 32;
    public static final int CALIB_FIX_K2 = 64;
    public static final int CALIB_FIX_K3 = 128;
    public static final int CALIB_FIX_K4 = 2048;
    public static final int CALIB_FIX_K5 = 4096;
    public static final int CALIB_FIX_K6 = 8192;
    public static final int CALIB_FIX_PRINCIPAL_POINT = 4;
    public static final int CALIB_FIX_S1_S2_S3_S4 = 65536;
    public static final int CALIB_FIX_TANGENT_DIST = 2097152;
    public static final int CALIB_FIX_TAUX_TAUY = 524288;
    public static final int CALIB_NINTRINSIC = 18;
    public static final int CALIB_RATIONAL_MODEL = 16384;
    public static final int CALIB_SAME_FOCAL_LENGTH = 512;
    public static final int CALIB_THIN_PRISM_MODEL = 32768;
    public static final int CALIB_TILTED_MODEL = 262144;
    public static final int CALIB_USE_EXTRINSIC_GUESS = 4194304;
    public static final int CALIB_USE_INTRINSIC_GUESS = 1;
    public static final int CALIB_USE_LU = 131072;
    public static final int CALIB_USE_QR = 1048576;
    public static final int CALIB_ZERO_DISPARITY = 1024;
    public static final int CALIB_ZERO_TANGENT_DIST = 8;
    public static final int CV_CALIB_CB_ADAPTIVE_THRESH = 1;
    public static final int CV_CALIB_CB_FAST_CHECK = 8;
    public static final int CV_CALIB_CB_FILTER_QUADS = 4;
    public static final int CV_CALIB_CB_NORMALIZE_IMAGE = 2;
    public static final int CV_CALIB_FIX_ASPECT_RATIO = 2;
    public static final int CV_CALIB_FIX_FOCAL_LENGTH = 16;
    public static final int CV_CALIB_FIX_INTRINSIC = 256;
    public static final int CV_CALIB_FIX_K1 = 32;
    public static final int CV_CALIB_FIX_K2 = 64;
    public static final int CV_CALIB_FIX_K3 = 128;
    public static final int CV_CALIB_FIX_K4 = 2048;
    public static final int CV_CALIB_FIX_K5 = 4096;
    public static final int CV_CALIB_FIX_K6 = 8192;
    public static final int CV_CALIB_FIX_PRINCIPAL_POINT = 4;
    public static final int CV_CALIB_FIX_S1_S2_S3_S4 = 65536;
    public static final int CV_CALIB_FIX_TANGENT_DIST = 2097152;
    public static final int CV_CALIB_FIX_TAUX_TAUY = 524288;
    public static final int CV_CALIB_NINTRINSIC = 18;
    public static final int CV_CALIB_RATIONAL_MODEL = 16384;
    public static final int CV_CALIB_SAME_FOCAL_LENGTH = 512;
    public static final int CV_CALIB_THIN_PRISM_MODEL = 32768;
    public static final int CV_CALIB_TILTED_MODEL = 262144;
    public static final int CV_CALIB_USE_INTRINSIC_GUESS = 1;
    public static final int CV_CALIB_ZERO_DISPARITY = 1024;
    public static final int CV_CALIB_ZERO_TANGENT_DIST = 8;
    public static final int CV_DLS = 3;
    public static final int CV_EPNP = 1;
    public static final int CV_FM_7POINT = 1;
    public static final int CV_FM_8POINT = 2;
    public static final int CV_FM_LMEDS = 4;
    public static final int CV_FM_LMEDS_ONLY = 4;
    public static final int CV_FM_RANSAC = 8;
    public static final int CV_FM_RANSAC_ONLY = 8;
    public static final int CV_ITERATIVE = 0;
    public static final int CV_LMEDS = 4;
    public static final int CV_P3P = 2;
    public static final int CV_RANSAC = 8;
    public static final int CV_STEREO_BM_NORMALIZED_RESPONSE = 0;
    public static final int CV_STEREO_BM_XSOBEL = 1;
    public static final int FISHEYE_CALIB_CHECK_COND = 4;
    public static final int FISHEYE_CALIB_FIX_INTRINSIC = 256;
    public static final int FISHEYE_CALIB_FIX_K1 = 16;
    public static final int FISHEYE_CALIB_FIX_K2 = 32;
    public static final int FISHEYE_CALIB_FIX_K3 = 64;
    public static final int FISHEYE_CALIB_FIX_K4 = 128;
    public static final int FISHEYE_CALIB_FIX_PRINCIPAL_POINT = 512;
    public static final int FISHEYE_CALIB_FIX_SKEW = 8;
    public static final int FISHEYE_CALIB_RECOMPUTE_EXTRINSIC = 2;
    public static final int FISHEYE_CALIB_USE_INTRINSIC_GUESS = 1;
    public static final int FM_7POINT = 1;
    public static final int FM_8POINT = 2;
    public static final int FM_LMEDS = 4;
    public static final int FM_RANSAC = 8;
    public static final int LMEDS = 4;
    public static final int PROJ_SPHERICAL_EQRECT = 1;
    public static final int PROJ_SPHERICAL_ORTHO = 0;
    public static final int RANSAC = 8;
    public static final int RHO = 16;
    public static final int SOLVEPNP_AP3P = 5;
    public static final int SOLVEPNP_DLS = 3;
    public static final int SOLVEPNP_EPNP = 1;
    public static final int SOLVEPNP_ITERATIVE = 0;
    public static final int SOLVEPNP_MAX_COUNT = 6;
    public static final int SOLVEPNP_P3P = 2;
    public static final int SOLVEPNP_UPNP = 4;

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Point3d RQDecomp3x3(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Point3d RQDecomp3x3(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat4, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat5, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat6);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Point3d RQDecomp3x3(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Point3d RQDecomp3x3(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat4, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat5, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat6);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Point3d RQDecomp3x3(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Point3d RQDecomp3x3(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat4, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat5, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat6);

    @Namespace("cv")
    public static native void Rodrigues(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv")
    public static native void Rodrigues(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat3);

    @Namespace("cv")
    public static native void Rodrigues(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv")
    public static native void Rodrigues(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat3);

    @Namespace("cv")
    public static native void Rodrigues(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv")
    public static native void Rodrigues(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat3);

    @Namespace("cv::fisheye")
    public static native double calibrate(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMatVector gpuMatVector3, @ByVal opencv_core.GpuMatVector gpuMatVector4);

    @Namespace("cv::fisheye")
    public static native double calibrate(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMatVector gpuMatVector3, @ByVal opencv_core.GpuMatVector gpuMatVector4, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::fisheye")
    public static native double calibrate(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.GpuMatVector gpuMatVector3, @ByVal opencv_core.GpuMatVector gpuMatVector4);

    @Namespace("cv::fisheye")
    public static native double calibrate(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.GpuMatVector gpuMatVector3, @ByVal opencv_core.GpuMatVector gpuMatVector4, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::fisheye")
    public static native double calibrate(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.GpuMatVector gpuMatVector3, @ByVal opencv_core.GpuMatVector gpuMatVector4);

    @Namespace("cv::fisheye")
    public static native double calibrate(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.GpuMatVector gpuMatVector3, @ByVal opencv_core.GpuMatVector gpuMatVector4, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::fisheye")
    public static native double calibrate(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.MatVector matVector3, @ByVal opencv_core.MatVector matVector4);

    @Namespace("cv::fisheye")
    public static native double calibrate(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.MatVector matVector3, @ByVal opencv_core.MatVector matVector4, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::fisheye")
    public static native double calibrate(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.MatVector matVector3, @ByVal opencv_core.MatVector matVector4);

    @Namespace("cv::fisheye")
    public static native double calibrate(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.MatVector matVector3, @ByVal opencv_core.MatVector matVector4, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::fisheye")
    public static native double calibrate(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.MatVector matVector3, @ByVal opencv_core.MatVector matVector4);

    @Namespace("cv::fisheye")
    public static native double calibrate(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.MatVector matVector3, @ByVal opencv_core.MatVector matVector4, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::fisheye")
    public static native double calibrate(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.UMatVector uMatVector3, @ByVal opencv_core.UMatVector uMatVector4);

    @Namespace("cv::fisheye")
    public static native double calibrate(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.UMatVector uMatVector3, @ByVal opencv_core.UMatVector uMatVector4, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::fisheye")
    public static native double calibrate(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.UMatVector uMatVector3, @ByVal opencv_core.UMatVector uMatVector4);

    @Namespace("cv::fisheye")
    public static native double calibrate(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.UMatVector uMatVector3, @ByVal opencv_core.UMatVector uMatVector4, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv::fisheye")
    public static native double calibrate(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMatVector uMatVector3, @ByVal opencv_core.UMatVector uMatVector4);

    @Namespace("cv::fisheye")
    public static native double calibrate(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMatVector uMatVector3, @ByVal opencv_core.UMatVector uMatVector4, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv")
    public static native double calibrateCamera(@ByVal opencv_core.Point3fVectorVector point3fVectorVector, @ByVal opencv_core.Point2fVectorVector point2fVectorVector, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2);

    @Namespace("cv")
    public static native double calibrateCamera(@ByVal opencv_core.Point3fVectorVector point3fVectorVector, @ByVal opencv_core.Point2fVectorVector point2fVectorVector, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv")
    @Name({"calibrateCamera"})
    public static native double calibrateCameraExtended(@ByVal opencv_core.Point3fVectorVector point3fVectorVector, @ByVal opencv_core.Point2fVectorVector point2fVectorVector, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5);

    @Namespace("cv")
    @Name({"calibrateCamera"})
    public static native double calibrateCameraExtended(@ByVal opencv_core.Point3fVectorVector point3fVectorVector, @ByVal opencv_core.Point2fVectorVector point2fVectorVector, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv")
    public static native double calibrateCameraRO(@ByVal opencv_core.Point3fVectorVector point3fVectorVector, @ByVal opencv_core.Point2fVectorVector point2fVectorVector, @ByVal opencv_core.Size size, int i, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv")
    public static native double calibrateCameraRO(@ByVal opencv_core.Point3fVectorVector point3fVectorVector, @ByVal opencv_core.Point2fVectorVector point2fVectorVector, @ByVal opencv_core.Size size, int i, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.Mat mat3, int i2, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv")
    @Name({"calibrateCameraRO"})
    public static native double calibrateCameraROExtended(@ByVal opencv_core.Point3fVectorVector point3fVectorVector, @ByVal opencv_core.Point2fVectorVector point2fVectorVector, @ByVal opencv_core.Size size, int i, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, @ByVal opencv_core.Mat mat7);

    @Namespace("cv")
    @Name({"calibrateCameraRO"})
    public static native double calibrateCameraROExtended(@ByVal opencv_core.Point3fVectorVector point3fVectorVector, @ByVal opencv_core.Point2fVectorVector point2fVectorVector, @ByVal opencv_core.Size size, int i, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, @ByVal opencv_core.Mat mat7, int i2, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv")
    public static native void calibrationMatrixValues(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.Size size, double d, double d2, @ByRef DoubleBuffer doubleBuffer, @ByRef DoubleBuffer doubleBuffer2, @ByRef DoubleBuffer doubleBuffer3, @ByRef opencv_core.Point2d point2d, @ByRef DoubleBuffer doubleBuffer4);

    @Namespace("cv")
    public static native void calibrationMatrixValues(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.Size size, double d, double d2, @ByRef DoublePointer doublePointer, @ByRef DoublePointer doublePointer2, @ByRef DoublePointer doublePointer3, @ByRef opencv_core.Point2d point2d, @ByRef DoublePointer doublePointer4);

    @Namespace("cv")
    public static native void calibrationMatrixValues(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.Size size, double d, double d2, @ByRef double[] dArr, @ByRef double[] dArr2, @ByRef double[] dArr3, @ByRef opencv_core.Point2d point2d, @ByRef double[] dArr4);

    @Namespace("cv")
    public static native void calibrationMatrixValues(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Size size, double d, double d2, @ByRef DoubleBuffer doubleBuffer, @ByRef DoubleBuffer doubleBuffer2, @ByRef DoubleBuffer doubleBuffer3, @ByRef opencv_core.Point2d point2d, @ByRef DoubleBuffer doubleBuffer4);

    @Namespace("cv")
    public static native void calibrationMatrixValues(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Size size, double d, double d2, @ByRef DoublePointer doublePointer, @ByRef DoublePointer doublePointer2, @ByRef DoublePointer doublePointer3, @ByRef opencv_core.Point2d point2d, @ByRef DoublePointer doublePointer4);

    @Namespace("cv")
    public static native void calibrationMatrixValues(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Size size, double d, double d2, @ByRef double[] dArr, @ByRef double[] dArr2, @ByRef double[] dArr3, @ByRef opencv_core.Point2d point2d, @ByRef double[] dArr4);

    @Namespace("cv")
    public static native void calibrationMatrixValues(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.Size size, double d, double d2, @ByRef DoubleBuffer doubleBuffer, @ByRef DoubleBuffer doubleBuffer2, @ByRef DoubleBuffer doubleBuffer3, @ByRef opencv_core.Point2d point2d, @ByRef DoubleBuffer doubleBuffer4);

    @Namespace("cv")
    public static native void calibrationMatrixValues(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.Size size, double d, double d2, @ByRef DoublePointer doublePointer, @ByRef DoublePointer doublePointer2, @ByRef DoublePointer doublePointer3, @ByRef opencv_core.Point2d point2d, @ByRef DoublePointer doublePointer4);

    @Namespace("cv")
    public static native void calibrationMatrixValues(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.Size size, double d, double d2, @ByRef double[] dArr, @ByRef double[] dArr2, @ByRef double[] dArr3, @ByRef opencv_core.Point2d point2d, @ByRef double[] dArr4);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean checkChessboard(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.Size size);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean checkChessboard(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Size size);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean checkChessboard(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.Size size);

    @Namespace("cv")
    public static native void composeRT(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6);

    @Namespace("cv")
    public static native void composeRT(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat7, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat8, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat9, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat10, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat11, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat12, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat13, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat14);

    @Namespace("cv")
    public static native void composeRT(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6);

    @Namespace("cv")
    public static native void composeRT(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat7, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat8, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat9, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat10, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat11, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat12, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat13, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat14);

    @Namespace("cv")
    public static native void composeRT(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6);

    @Namespace("cv")
    public static native void composeRT(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat7, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat8, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat9, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat10, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat11, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat12, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat13, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat14);

    @Namespace("cv")
    public static native void computeCorrespondEpilines(@ByVal opencv_core.GpuMat gpuMat, int i, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv")
    public static native void computeCorrespondEpilines(@ByVal opencv_core.Mat mat, int i, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv")
    public static native void computeCorrespondEpilines(@ByVal opencv_core.UMat uMat, int i, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv")
    public static native void convertPointsFromHomogeneous(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv")
    public static native void convertPointsFromHomogeneous(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv")
    public static native void convertPointsFromHomogeneous(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv")
    public static native void convertPointsHomogeneous(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv")
    public static native void convertPointsHomogeneous(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv")
    public static native void convertPointsHomogeneous(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv")
    public static native void convertPointsToHomogeneous(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv")
    public static native void convertPointsToHomogeneous(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv")
    public static native void convertPointsToHomogeneous(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv")
    public static native void correctMatches(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5);

    @Namespace("cv")
    public static native void correctMatches(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5);

    @Namespace("cv")
    public static native void correctMatches(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5);

    @Namespace("cv")
    public static native void decomposeEssentialMat(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4);

    @Namespace("cv")
    public static native void decomposeEssentialMat(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4);

    @Namespace("cv")
    public static native void decomposeEssentialMat(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4);

    @Namespace("cv")
    public static native int decomposeHomographyMat(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByVal opencv_core.GpuMatVector gpuMatVector3);

    @Namespace("cv")
    public static native int decomposeHomographyMat(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.MatVector matVector3);

    @Namespace("cv")
    public static native int decomposeHomographyMat(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @ByVal opencv_core.UMatVector uMatVector3);

    @Namespace("cv")
    public static native int decomposeHomographyMat(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByVal opencv_core.GpuMatVector gpuMatVector3);

    @Namespace("cv")
    public static native int decomposeHomographyMat(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.MatVector matVector3);

    @Namespace("cv")
    public static native int decomposeHomographyMat(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @ByVal opencv_core.UMatVector uMatVector3);

    @Namespace("cv")
    public static native int decomposeHomographyMat(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByVal opencv_core.GpuMatVector gpuMatVector3);

    @Namespace("cv")
    public static native int decomposeHomographyMat(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.MatVector matVector3);

    @Namespace("cv")
    public static native int decomposeHomographyMat(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @ByVal opencv_core.UMatVector uMatVector3);

    @Namespace("cv")
    public static native void decomposeProjectionMatrix(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4);

    @Namespace("cv")
    public static native void decomposeProjectionMatrix(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat5, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat6, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat7, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat8);

    @Namespace("cv")
    public static native void decomposeProjectionMatrix(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4);

    @Namespace("cv")
    public static native void decomposeProjectionMatrix(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat5, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat6, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat7, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat8);

    @Namespace("cv")
    public static native void decomposeProjectionMatrix(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4);

    @Namespace("cv")
    public static native void decomposeProjectionMatrix(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat5, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat6, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat7, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat8);

    @Namespace("cv::fisheye")
    public static native void distortPoints(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4);

    @Namespace("cv::fisheye")
    public static native void distortPoints(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, double d);

    @Namespace("cv::fisheye")
    public static native void distortPoints(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4);

    @Namespace("cv::fisheye")
    public static native void distortPoints(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, double d);

    @Namespace("cv::fisheye")
    public static native void distortPoints(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4);

    @Namespace("cv::fisheye")
    public static native void distortPoints(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, double d);

    @Namespace("cv")
    public static native void drawChessboardCorners(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat2, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void drawChessboardCorners(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat2, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void drawChessboardCorners(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat2, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void drawFrameAxes(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, float f);

    @Namespace("cv")
    public static native void drawFrameAxes(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, float f, int i);

    @Namespace("cv")
    public static native void drawFrameAxes(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, float f);

    @Namespace("cv")
    public static native void drawFrameAxes(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, float f, int i);

    @Namespace("cv")
    public static native void drawFrameAxes(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, float f);

    @Namespace("cv")
    public static native void drawFrameAxes(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, float f, int i);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat estimateAffine2D(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat estimateAffine2D(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat3, int i, double d, @Cast({"size_t"}) long j, double d2, @Cast({"size_t"}) long j2);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat estimateAffine2D(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat estimateAffine2D(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat3, int i, double d, @Cast({"size_t"}) long j, double d2, @Cast({"size_t"}) long j2);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat estimateAffine2D(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat estimateAffine2D(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat3, int i, double d, @Cast({"size_t"}) long j, double d2, @Cast({"size_t"}) long j2);

    @Namespace("cv")
    public static native int estimateAffine3D(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4);

    @Namespace("cv")
    public static native int estimateAffine3D(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, double d, double d2);

    @Namespace("cv")
    public static native int estimateAffine3D(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4);

    @Namespace("cv")
    public static native int estimateAffine3D(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, double d, double d2);

    @Namespace("cv")
    public static native int estimateAffine3D(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4);

    @Namespace("cv")
    public static native int estimateAffine3D(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, double d, double d2);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat estimateAffinePartial2D(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat estimateAffinePartial2D(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat3, int i, double d, @Cast({"size_t"}) long j, double d2, @Cast({"size_t"}) long j2);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat estimateAffinePartial2D(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat estimateAffinePartial2D(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat3, int i, double d, @Cast({"size_t"}) long j, double d2, @Cast({"size_t"}) long j2);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat estimateAffinePartial2D(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat estimateAffinePartial2D(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat3, int i, double d, @Cast({"size_t"}) long j, double d2, @Cast({"size_t"}) long j2);

    @Namespace("cv::fisheye")
    public static native void estimateNewCameraMatrixForUndistortRectify(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4);

    @Namespace("cv::fisheye")
    public static native void estimateNewCameraMatrixForUndistortRectify(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, double d, @ByRef(nullValue = "cv::Size()") @Const opencv_core.Size size2, double d2);

    @Namespace("cv::fisheye")
    public static native void estimateNewCameraMatrixForUndistortRectify(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4);

    @Namespace("cv::fisheye")
    public static native void estimateNewCameraMatrixForUndistortRectify(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, double d, @ByRef(nullValue = "cv::Size()") @Const opencv_core.Size size2, double d2);

    @Namespace("cv::fisheye")
    public static native void estimateNewCameraMatrixForUndistortRectify(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4);

    @Namespace("cv::fisheye")
    public static native void estimateNewCameraMatrixForUndistortRectify(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, double d, @ByRef(nullValue = "cv::Size()") @Const opencv_core.Size size2, double d2);

    @Namespace("cv")
    public static native void filterHomographyDecompByVisibleRefpoints(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv")
    public static native void filterHomographyDecompByVisibleRefpoints(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat4);

    @Namespace("cv")
    public static native void filterHomographyDecompByVisibleRefpoints(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv")
    public static native void filterHomographyDecompByVisibleRefpoints(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat4);

    @Namespace("cv")
    public static native void filterHomographyDecompByVisibleRefpoints(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv")
    public static native void filterHomographyDecompByVisibleRefpoints(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat4);

    @Namespace("cv")
    public static native void filterHomographyDecompByVisibleRefpoints(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv")
    public static native void filterHomographyDecompByVisibleRefpoints(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat4);

    @Namespace("cv")
    public static native void filterHomographyDecompByVisibleRefpoints(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv")
    public static native void filterHomographyDecompByVisibleRefpoints(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat4);

    @Namespace("cv")
    public static native void filterHomographyDecompByVisibleRefpoints(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv")
    public static native void filterHomographyDecompByVisibleRefpoints(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat4);

    @Namespace("cv")
    public static native void filterHomographyDecompByVisibleRefpoints(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv")
    public static native void filterHomographyDecompByVisibleRefpoints(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat4);

    @Namespace("cv")
    public static native void filterHomographyDecompByVisibleRefpoints(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv")
    public static native void filterHomographyDecompByVisibleRefpoints(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat4);

    @Namespace("cv")
    public static native void filterHomographyDecompByVisibleRefpoints(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv")
    public static native void filterHomographyDecompByVisibleRefpoints(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat4);

    @Namespace("cv")
    public static native void filterSpeckles(@ByVal opencv_core.GpuMat gpuMat, double d, int i, double d2);

    @Namespace("cv")
    public static native void filterSpeckles(@ByVal opencv_core.GpuMat gpuMat, double d, int i, double d2, @ByVal(nullValue = "cv::InputOutputArray(cv::noArray())") opencv_core.GpuMat gpuMat2);

    @Namespace("cv")
    public static native void filterSpeckles(@ByVal opencv_core.Mat mat, double d, int i, double d2);

    @Namespace("cv")
    public static native void filterSpeckles(@ByVal opencv_core.Mat mat, double d, int i, double d2, @ByVal(nullValue = "cv::InputOutputArray(cv::noArray())") opencv_core.Mat mat2);

    @Namespace("cv")
    public static native void filterSpeckles(@ByVal opencv_core.UMat uMat, double d, int i, double d2);

    @Namespace("cv")
    public static native void filterSpeckles(@ByVal opencv_core.UMat uMat, double d, int i, double d2, @ByVal(nullValue = "cv::InputOutputArray(cv::noArray())") opencv_core.UMat uMat2);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean find4QuadCornerSubpix(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.Size size);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean find4QuadCornerSubpix(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Size size);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean find4QuadCornerSubpix(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.Size size);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean findChessboardCorners(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean findChessboardCorners(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat2, int i);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean findChessboardCorners(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat2);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean findChessboardCorners(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat2, int i);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean findChessboardCorners(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean findChessboardCorners(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat2, int i);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean findChessboardCornersSB(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean findChessboardCornersSB(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat2, int i);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean findChessboardCornersSB(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat2);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean findChessboardCornersSB(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat2, int i);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean findChessboardCornersSB(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean findChessboardCornersSB(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat2, int i);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean findCirclesGrid(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean findCirclesGrid(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat2, int i, @opencv_core.Ptr @Cast({"cv::FeatureDetector*"}) opencv_features2d.Feature2D feature2D);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean findCirclesGrid(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat2, int i, @opencv_core.Ptr @Cast({"cv::FeatureDetector*"}) opencv_features2d.Feature2D feature2D, @ByRef @Const CirclesGridFinderParameters circlesGridFinderParameters);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean findCirclesGrid(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat2);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean findCirclesGrid(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat2, int i, @opencv_core.Ptr @Cast({"cv::FeatureDetector*"}) opencv_features2d.Feature2D feature2D);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean findCirclesGrid(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat2, int i, @opencv_core.Ptr @Cast({"cv::FeatureDetector*"}) opencv_features2d.Feature2D feature2D, @ByRef @Const CirclesGridFinderParameters circlesGridFinderParameters);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean findCirclesGrid(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean findCirclesGrid(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat2, int i, @opencv_core.Ptr @Cast({"cv::FeatureDetector*"}) opencv_features2d.Feature2D feature2D);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean findCirclesGrid(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat2, int i, @opencv_core.Ptr @Cast({"cv::FeatureDetector*"}) opencv_features2d.Feature2D feature2D, @ByRef @Const CirclesGridFinderParameters circlesGridFinderParameters);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findEssentialMat(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findEssentialMat(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, double d, @ByVal(nullValue = "cv::Point2d(0, 0)") opencv_core.Point2d point2d, int i, double d2, double d3, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat3);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findEssentialMat(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findEssentialMat(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, int i, double d, double d2, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat4);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findEssentialMat(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findEssentialMat(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, double d, @ByVal(nullValue = "cv::Point2d(0, 0)") opencv_core.Point2d point2d, int i, double d2, double d3, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat3);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findEssentialMat(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findEssentialMat(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, int i, double d, double d2, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat4);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findEssentialMat(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findEssentialMat(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, double d, @ByVal(nullValue = "cv::Point2d(0, 0)") opencv_core.Point2d point2d, int i, double d2, double d3, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat3);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findEssentialMat(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findEssentialMat(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, int i, double d, double d2, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat4);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findFundamentalMat(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findFundamentalMat(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, double d, double d2, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat3);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findFundamentalMat(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findFundamentalMat(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, int i, double d, double d2);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findFundamentalMat(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findFundamentalMat(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, double d, double d2, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat3);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findFundamentalMat(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findFundamentalMat(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, int i, double d, double d2);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findFundamentalMat(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findFundamentalMat(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, double d, double d2, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat3);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findFundamentalMat(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findFundamentalMat(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, int i, double d, double d2);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findHomography(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findHomography(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, double d, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat3, int i2, double d2);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findHomography(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findHomography(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, int i, double d);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findHomography(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findHomography(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, double d, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat3, int i2, double d2);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findHomography(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findHomography(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, int i, double d);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findHomography(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findHomography(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, double d, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat3, int i2, double d2);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findHomography(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat findHomography(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, int i, double d);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat getDefaultNewCameraMatrix(@ByVal opencv_core.GpuMat gpuMat);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat getDefaultNewCameraMatrix(@ByVal opencv_core.GpuMat gpuMat, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat getDefaultNewCameraMatrix(@ByVal opencv_core.Mat mat);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat getDefaultNewCameraMatrix(@ByVal opencv_core.Mat mat, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat getDefaultNewCameraMatrix(@ByVal opencv_core.UMat uMat);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat getDefaultNewCameraMatrix(@ByVal opencv_core.UMat uMat, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat getOptimalNewCameraMatrix(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.Size size, double d);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat getOptimalNewCameraMatrix(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.Size size, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, opencv_core.Rect rect, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat getOptimalNewCameraMatrix(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Size size, double d);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat getOptimalNewCameraMatrix(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Size size, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, opencv_core.Rect rect, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat getOptimalNewCameraMatrix(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.Size size, double d);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat getOptimalNewCameraMatrix(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.Size size, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, opencv_core.Rect rect, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Rect getValidDisparityROI(@ByVal opencv_core.Rect rect, @ByVal opencv_core.Rect rect2, int i, int i2, int i3);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat initCameraMatrix2D(@ByVal opencv_core.Point3fVectorVector point3fVectorVector, @ByVal opencv_core.Point2fVectorVector point2fVectorVector, @ByVal opencv_core.Size size);

    @Namespace("cv")
    @ByVal
    public static native opencv_core.Mat initCameraMatrix2D(@ByVal opencv_core.Point3fVectorVector point3fVectorVector, @ByVal opencv_core.Point2fVectorVector point2fVectorVector, @ByVal opencv_core.Size size, double d);

    @Namespace("cv")
    public static native void initUndistortRectifyMap(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.Size size, int i, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6);

    @Namespace("cv")
    public static native void initUndistortRectifyMap(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Size size, int i, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6);

    @Namespace("cv")
    public static native void initUndistortRectifyMap(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.Size size, int i, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6);

    @Namespace("cv")
    public static native float initWideAngleProjMap(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.Size size, int i, int i2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4);

    @Namespace("cv")
    public static native float initWideAngleProjMap(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.Size size, int i, int i2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, int i3);

    @Namespace("cv")
    public static native float initWideAngleProjMap(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.Size size, int i, int i2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @Cast({"cv::UndistortTypes"}) int i3, double d);

    @Namespace("cv")
    public static native float initWideAngleProjMap(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Size size, int i, int i2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4);

    @Namespace("cv")
    public static native float initWideAngleProjMap(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Size size, int i, int i2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, int i3);

    @Namespace("cv")
    public static native float initWideAngleProjMap(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Size size, int i, int i2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @Cast({"cv::UndistortTypes"}) int i3, double d);

    @Namespace("cv")
    public static native float initWideAngleProjMap(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.Size size, int i, int i2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4);

    @Namespace("cv")
    public static native float initWideAngleProjMap(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.Size size, int i, int i2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, int i3);

    @Namespace("cv")
    public static native float initWideAngleProjMap(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.Size size, int i, int i2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @Cast({"cv::UndistortTypes"}) int i3, double d);

    @Namespace("cv")
    public static native void matMulDeriv(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4);

    @Namespace("cv")
    public static native void matMulDeriv(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4);

    @Namespace("cv")
    public static native void matMulDeriv(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4);

    @Namespace("cv")
    public static native void projectPoints(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6);

    @Namespace("cv::fisheye")
    public static native void projectPoints(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6, double d, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat7);

    @Namespace("cv")
    public static native void projectPoints(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat7, double d);

    @Namespace("cv::fisheye")
    public static native void projectPoints(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef @Const opencv_core.Mat mat, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4);

    @Namespace("cv::fisheye")
    public static native void projectPoints(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef @Const opencv_core.Mat mat, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, double d, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat5);

    @Namespace("cv::fisheye")
    public static native void projectPoints(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef @Const opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5);

    @Namespace("cv::fisheye")
    public static native void projectPoints(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef @Const opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, double d, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat6);

    @Namespace("cv")
    public static native void projectPoints(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6);

    @Namespace("cv::fisheye")
    public static native void projectPoints(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, double d, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat7);

    @Namespace("cv")
    public static native void projectPoints(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat7, double d);

    @Namespace("cv::fisheye")
    public static native void projectPoints(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef @Const opencv_core.Mat mat, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4);

    @Namespace("cv::fisheye")
    public static native void projectPoints(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef @Const opencv_core.Mat mat, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, double d, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat5);

    @Namespace("cv")
    public static native void projectPoints(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6);

    @Namespace("cv::fisheye")
    public static native void projectPoints(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6, double d, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat7);

    @Namespace("cv")
    public static native void projectPoints(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat7, double d);

    @Namespace("cv")
    public static native int recoverPose(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5);

    @Namespace("cv")
    public static native int recoverPose(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, double d, @ByVal(nullValue = "cv::Point2d(0, 0)") opencv_core.Point2d point2d, @ByVal(nullValue = "cv::InputOutputArray(cv::noArray())") opencv_core.GpuMat gpuMat6);

    @Namespace("cv")
    public static native int recoverPose(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6);

    @Namespace("cv")
    public static native int recoverPose(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6, double d);

    @Namespace("cv")
    public static native int recoverPose(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6, double d, @ByVal(nullValue = "cv::InputOutputArray(cv::noArray())") opencv_core.GpuMat gpuMat7, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat8);

    @Namespace("cv")
    public static native int recoverPose(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6, @ByVal(nullValue = "cv::InputOutputArray(cv::noArray())") opencv_core.GpuMat gpuMat7);

    @Namespace("cv")
    public static native int recoverPose(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5);

    @Namespace("cv")
    public static native int recoverPose(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, double d, @ByVal(nullValue = "cv::Point2d(0, 0)") opencv_core.Point2d point2d, @ByVal(nullValue = "cv::InputOutputArray(cv::noArray())") opencv_core.Mat mat6);

    @Namespace("cv")
    public static native int recoverPose(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6);

    @Namespace("cv")
    public static native int recoverPose(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, double d);

    @Namespace("cv")
    public static native int recoverPose(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, double d, @ByVal(nullValue = "cv::InputOutputArray(cv::noArray())") opencv_core.Mat mat7, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat8);

    @Namespace("cv")
    public static native int recoverPose(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, @ByVal(nullValue = "cv::InputOutputArray(cv::noArray())") opencv_core.Mat mat7);

    @Namespace("cv")
    public static native int recoverPose(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5);

    @Namespace("cv")
    public static native int recoverPose(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, double d, @ByVal(nullValue = "cv::Point2d(0, 0)") opencv_core.Point2d point2d, @ByVal(nullValue = "cv::InputOutputArray(cv::noArray())") opencv_core.UMat uMat6);

    @Namespace("cv")
    public static native int recoverPose(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6);

    @Namespace("cv")
    public static native int recoverPose(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6, double d);

    @Namespace("cv")
    public static native int recoverPose(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6, double d, @ByVal(nullValue = "cv::InputOutputArray(cv::noArray())") opencv_core.UMat uMat7, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat8);

    @Namespace("cv")
    public static native int recoverPose(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6, @ByVal(nullValue = "cv::InputOutputArray(cv::noArray())") opencv_core.UMat uMat7);

    @Namespace("cv")
    public static native float rectify3Collinear(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat7, @ByVal opencv_core.GpuMat gpuMat8, @ByVal opencv_core.GpuMat gpuMat9, @ByVal opencv_core.GpuMat gpuMat10, @ByVal opencv_core.GpuMat gpuMat11, @ByVal opencv_core.GpuMat gpuMat12, @ByVal opencv_core.GpuMat gpuMat13, @ByVal opencv_core.GpuMat gpuMat14, @ByVal opencv_core.GpuMat gpuMat15, @ByVal opencv_core.GpuMat gpuMat16, @ByVal opencv_core.GpuMat gpuMat17, double d, @ByVal opencv_core.Size size2, opencv_core.Rect rect, opencv_core.Rect rect2, int i);

    @Namespace("cv")
    public static native float rectify3Collinear(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat7, @ByVal opencv_core.GpuMat gpuMat8, @ByVal opencv_core.GpuMat gpuMat9, @ByVal opencv_core.GpuMat gpuMat10, @ByVal opencv_core.GpuMat gpuMat11, @ByVal opencv_core.GpuMat gpuMat12, @ByVal opencv_core.GpuMat gpuMat13, @ByVal opencv_core.GpuMat gpuMat14, @ByVal opencv_core.GpuMat gpuMat15, @ByVal opencv_core.GpuMat gpuMat16, @ByVal opencv_core.GpuMat gpuMat17, double d, @ByVal opencv_core.Size size2, opencv_core.Rect rect, opencv_core.Rect rect2, int i);

    @Namespace("cv")
    public static native float rectify3Collinear(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat7, @ByVal opencv_core.GpuMat gpuMat8, @ByVal opencv_core.GpuMat gpuMat9, @ByVal opencv_core.GpuMat gpuMat10, @ByVal opencv_core.GpuMat gpuMat11, @ByVal opencv_core.GpuMat gpuMat12, @ByVal opencv_core.GpuMat gpuMat13, @ByVal opencv_core.GpuMat gpuMat14, @ByVal opencv_core.GpuMat gpuMat15, @ByVal opencv_core.GpuMat gpuMat16, @ByVal opencv_core.GpuMat gpuMat17, double d, @ByVal opencv_core.Size size2, opencv_core.Rect rect, opencv_core.Rect rect2, int i);

    @Namespace("cv")
    public static native float rectify3Collinear(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat7, @ByVal opencv_core.Mat mat8, @ByVal opencv_core.Mat mat9, @ByVal opencv_core.Mat mat10, @ByVal opencv_core.Mat mat11, @ByVal opencv_core.Mat mat12, @ByVal opencv_core.Mat mat13, @ByVal opencv_core.Mat mat14, @ByVal opencv_core.Mat mat15, @ByVal opencv_core.Mat mat16, @ByVal opencv_core.Mat mat17, double d, @ByVal opencv_core.Size size2, opencv_core.Rect rect, opencv_core.Rect rect2, int i);

    @Namespace("cv")
    public static native float rectify3Collinear(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat7, @ByVal opencv_core.Mat mat8, @ByVal opencv_core.Mat mat9, @ByVal opencv_core.Mat mat10, @ByVal opencv_core.Mat mat11, @ByVal opencv_core.Mat mat12, @ByVal opencv_core.Mat mat13, @ByVal opencv_core.Mat mat14, @ByVal opencv_core.Mat mat15, @ByVal opencv_core.Mat mat16, @ByVal opencv_core.Mat mat17, double d, @ByVal opencv_core.Size size2, opencv_core.Rect rect, opencv_core.Rect rect2, int i);

    @Namespace("cv")
    public static native float rectify3Collinear(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat7, @ByVal opencv_core.Mat mat8, @ByVal opencv_core.Mat mat9, @ByVal opencv_core.Mat mat10, @ByVal opencv_core.Mat mat11, @ByVal opencv_core.Mat mat12, @ByVal opencv_core.Mat mat13, @ByVal opencv_core.Mat mat14, @ByVal opencv_core.Mat mat15, @ByVal opencv_core.Mat mat16, @ByVal opencv_core.Mat mat17, double d, @ByVal opencv_core.Size size2, opencv_core.Rect rect, opencv_core.Rect rect2, int i);

    @Namespace("cv")
    public static native float rectify3Collinear(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat7, @ByVal opencv_core.UMat uMat8, @ByVal opencv_core.UMat uMat9, @ByVal opencv_core.UMat uMat10, @ByVal opencv_core.UMat uMat11, @ByVal opencv_core.UMat uMat12, @ByVal opencv_core.UMat uMat13, @ByVal opencv_core.UMat uMat14, @ByVal opencv_core.UMat uMat15, @ByVal opencv_core.UMat uMat16, @ByVal opencv_core.UMat uMat17, double d, @ByVal opencv_core.Size size2, opencv_core.Rect rect, opencv_core.Rect rect2, int i);

    @Namespace("cv")
    public static native float rectify3Collinear(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat7, @ByVal opencv_core.UMat uMat8, @ByVal opencv_core.UMat uMat9, @ByVal opencv_core.UMat uMat10, @ByVal opencv_core.UMat uMat11, @ByVal opencv_core.UMat uMat12, @ByVal opencv_core.UMat uMat13, @ByVal opencv_core.UMat uMat14, @ByVal opencv_core.UMat uMat15, @ByVal opencv_core.UMat uMat16, @ByVal opencv_core.UMat uMat17, double d, @ByVal opencv_core.Size size2, opencv_core.Rect rect, opencv_core.Rect rect2, int i);

    @Namespace("cv")
    public static native float rectify3Collinear(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat7, @ByVal opencv_core.UMat uMat8, @ByVal opencv_core.UMat uMat9, @ByVal opencv_core.UMat uMat10, @ByVal opencv_core.UMat uMat11, @ByVal opencv_core.UMat uMat12, @ByVal opencv_core.UMat uMat13, @ByVal opencv_core.UMat uMat14, @ByVal opencv_core.UMat uMat15, @ByVal opencv_core.UMat uMat16, @ByVal opencv_core.UMat uMat17, double d, @ByVal opencv_core.Size size2, opencv_core.Rect rect, opencv_core.Rect rect2, int i);

    @Namespace("cv")
    public static native void reprojectImageTo3D(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv")
    public static native void reprojectImageTo3D(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @Cast({"bool"}) boolean z, int i);

    @Namespace("cv")
    public static native void reprojectImageTo3D(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv")
    public static native void reprojectImageTo3D(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @Cast({"bool"}) boolean z, int i);

    @Namespace("cv")
    public static native void reprojectImageTo3D(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv")
    public static native void reprojectImageTo3D(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @Cast({"bool"}) boolean z, int i);

    @Namespace("cv")
    public static native double sampsonDistance(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv")
    public static native double sampsonDistance(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

    @Namespace("cv")
    public static native double sampsonDistance(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv")
    public static native int solveP3P(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, int i);

    @Namespace("cv")
    public static native int solveP3P(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, int i);

    @Namespace("cv")
    public static native int solveP3P(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, int i);

    @Namespace("cv")
    public static native int solveP3P(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, int i);

    @Namespace("cv")
    public static native int solveP3P(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, int i);

    @Namespace("cv")
    public static native int solveP3P(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, int i);

    @Namespace("cv")
    public static native int solveP3P(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, int i);

    @Namespace("cv")
    public static native int solveP3P(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, int i);

    @Namespace("cv")
    public static native int solveP3P(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, int i);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean solvePnP(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean solvePnP(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6, @Cast({"bool"}) boolean z, int i);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean solvePnP(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean solvePnP(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, @Cast({"bool"}) boolean z, int i);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean solvePnP(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean solvePnP(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6, @Cast({"bool"}) boolean z, int i);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean solvePnPRansac(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean solvePnPRansac(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6, @Cast({"bool"}) boolean z, int i, float f, double d, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat7, int i2);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean solvePnPRansac(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean solvePnPRansac(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, @Cast({"bool"}) boolean z, int i, float f, double d, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat7, int i2);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean solvePnPRansac(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean solvePnPRansac(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6, @Cast({"bool"}) boolean z, int i, float f, double d, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat7, int i2);

    @Namespace("cv")
    public static native double stereoCalibrate(@ByVal opencv_core.Point3fVectorVector point3fVectorVector, @ByVal opencv_core.Point2fVectorVector point2fVectorVector, @ByVal opencv_core.Point2fVectorVector point2fVectorVector2, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, @ByVal opencv_core.Mat mat7, @ByVal opencv_core.Mat mat8);

    @Namespace("cv")
    public static native double stereoCalibrate(@ByVal opencv_core.Point3fVectorVector point3fVectorVector, @ByVal opencv_core.Point2fVectorVector point2fVectorVector, @ByVal opencv_core.Point2fVectorVector point2fVectorVector2, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, @ByVal opencv_core.Mat mat7, @ByVal opencv_core.Mat mat8, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 1e-6)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv")
    @Name({"stereoCalibrate"})
    public static native double stereoCalibrateExtended(@ByVal opencv_core.Point3fVectorVector point3fVectorVector, @ByVal opencv_core.Point2fVectorVector point2fVectorVector, @ByVal opencv_core.Point2fVectorVector point2fVectorVector2, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, @ByVal opencv_core.Mat mat7, @ByVal opencv_core.Mat mat8, @ByVal opencv_core.Mat mat9);

    @Namespace("cv")
    @Name({"stereoCalibrate"})
    public static native double stereoCalibrateExtended(@ByVal opencv_core.Point3fVectorVector point3fVectorVector, @ByVal opencv_core.Point2fVectorVector point2fVectorVector, @ByVal opencv_core.Point2fVectorVector point2fVectorVector2, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, @ByVal opencv_core.Mat mat7, @ByVal opencv_core.Mat mat8, @ByVal opencv_core.Mat mat9, int i, @ByVal(nullValue = "cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 1e-6)") opencv_core.TermCriteria termCriteria);

    @Namespace("cv")
    public static native void stereoRectify(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6, @ByVal opencv_core.GpuMat gpuMat7, @ByVal opencv_core.GpuMat gpuMat8, @ByVal opencv_core.GpuMat gpuMat9, @ByVal opencv_core.GpuMat gpuMat10, @ByVal opencv_core.GpuMat gpuMat11);

    @Namespace("cv::fisheye")
    public static native void stereoRectify(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6, @ByVal opencv_core.GpuMat gpuMat7, @ByVal opencv_core.GpuMat gpuMat8, @ByVal opencv_core.GpuMat gpuMat9, @ByVal opencv_core.GpuMat gpuMat10, @ByVal opencv_core.GpuMat gpuMat11, int i);

    @Namespace("cv")
    public static native void stereoRectify(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6, @ByVal opencv_core.GpuMat gpuMat7, @ByVal opencv_core.GpuMat gpuMat8, @ByVal opencv_core.GpuMat gpuMat9, @ByVal opencv_core.GpuMat gpuMat10, @ByVal opencv_core.GpuMat gpuMat11, int i, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, opencv_core.Rect rect, opencv_core.Rect rect2);

    @Namespace("cv::fisheye")
    public static native void stereoRectify(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6, @ByVal opencv_core.GpuMat gpuMat7, @ByVal opencv_core.GpuMat gpuMat8, @ByVal opencv_core.GpuMat gpuMat9, @ByVal opencv_core.GpuMat gpuMat10, @ByVal opencv_core.GpuMat gpuMat11, int i, @ByRef(nullValue = "cv::Size()") @Const opencv_core.Size size2, double d, double d2);

    @Namespace("cv")
    public static native void stereoRectify(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, @ByVal opencv_core.Mat mat7, @ByVal opencv_core.Mat mat8, @ByVal opencv_core.Mat mat9, @ByVal opencv_core.Mat mat10, @ByVal opencv_core.Mat mat11);

    @Namespace("cv::fisheye")
    public static native void stereoRectify(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, @ByVal opencv_core.Mat mat7, @ByVal opencv_core.Mat mat8, @ByVal opencv_core.Mat mat9, @ByVal opencv_core.Mat mat10, @ByVal opencv_core.Mat mat11, int i);

    @Namespace("cv")
    public static native void stereoRectify(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, @ByVal opencv_core.Mat mat7, @ByVal opencv_core.Mat mat8, @ByVal opencv_core.Mat mat9, @ByVal opencv_core.Mat mat10, @ByVal opencv_core.Mat mat11, int i, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, opencv_core.Rect rect, opencv_core.Rect rect2);

    @Namespace("cv::fisheye")
    public static native void stereoRectify(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, @ByVal opencv_core.Mat mat7, @ByVal opencv_core.Mat mat8, @ByVal opencv_core.Mat mat9, @ByVal opencv_core.Mat mat10, @ByVal opencv_core.Mat mat11, int i, @ByRef(nullValue = "cv::Size()") @Const opencv_core.Size size2, double d, double d2);

    @Namespace("cv")
    public static native void stereoRectify(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6, @ByVal opencv_core.UMat uMat7, @ByVal opencv_core.UMat uMat8, @ByVal opencv_core.UMat uMat9, @ByVal opencv_core.UMat uMat10, @ByVal opencv_core.UMat uMat11);

    @Namespace("cv::fisheye")
    public static native void stereoRectify(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6, @ByVal opencv_core.UMat uMat7, @ByVal opencv_core.UMat uMat8, @ByVal opencv_core.UMat uMat9, @ByVal opencv_core.UMat uMat10, @ByVal opencv_core.UMat uMat11, int i);

    @Namespace("cv")
    public static native void stereoRectify(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6, @ByVal opencv_core.UMat uMat7, @ByVal opencv_core.UMat uMat8, @ByVal opencv_core.UMat uMat9, @ByVal opencv_core.UMat uMat10, @ByVal opencv_core.UMat uMat11, int i, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size2, opencv_core.Rect rect, opencv_core.Rect rect2);

    @Namespace("cv::fisheye")
    public static native void stereoRectify(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByRef @Const opencv_core.Size size, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6, @ByVal opencv_core.UMat uMat7, @ByVal opencv_core.UMat uMat8, @ByVal opencv_core.UMat uMat9, @ByVal opencv_core.UMat uMat10, @ByVal opencv_core.UMat uMat11, int i, @ByRef(nullValue = "cv::Size()") @Const opencv_core.Size size2, double d, double d2);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean stereoRectifyUncalibrated(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean stereoRectifyUncalibrated(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.Size size, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, double d);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean stereoRectifyUncalibrated(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean stereoRectifyUncalibrated(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Size size, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, double d);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean stereoRectifyUncalibrated(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5);

    @Namespace("cv")
    @Cast({"bool"})
    public static native boolean stereoRectifyUncalibrated(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.Size size, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, double d);

    @Namespace("cv")
    public static native void triangulatePoints(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5);

    @Namespace("cv")
    public static native void triangulatePoints(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5);

    @Namespace("cv")
    public static native void triangulatePoints(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5);

    @Namespace("cv")
    public static native void undistort(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4);

    @Namespace("cv")
    public static native void undistort(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat5);

    @Namespace("cv")
    public static native void undistort(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4);

    @Namespace("cv")
    public static native void undistort(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat5);

    @Namespace("cv")
    public static native void undistort(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4);

    @Namespace("cv")
    public static native void undistort(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat5);

    @Namespace("cv::fisheye")
    public static native void undistortImage(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4);

    @Namespace("cv::fisheye")
    public static native void undistortImage(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat5, @ByRef(nullValue = "cv::Size()") @Const opencv_core.Size size);

    @Namespace("cv::fisheye")
    public static native void undistortImage(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4);

    @Namespace("cv::fisheye")
    public static native void undistortImage(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat5, @ByRef(nullValue = "cv::Size()") @Const opencv_core.Size size);

    @Namespace("cv::fisheye")
    public static native void undistortImage(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4);

    @Namespace("cv::fisheye")
    public static native void undistortImage(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat5, @ByRef(nullValue = "cv::Size()") @Const opencv_core.Size size);

    @Namespace("cv")
    public static native void undistortPoints(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4);

    @Namespace("cv")
    public static native void undistortPoints(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat5, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat6);

    @Namespace("cv")
    public static native void undistortPoints(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4);

    @Namespace("cv")
    public static native void undistortPoints(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat5, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat6);

    @Namespace("cv")
    public static native void undistortPoints(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4);

    @Namespace("cv")
    public static native void undistortPoints(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat5, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat6);

    @Namespace("cv")
    @Name({"undistortPoints"})
    public static native void undistortPointsIter(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3, @ByVal opencv_core.GpuMat gpuMat4, @ByVal opencv_core.GpuMat gpuMat5, @ByVal opencv_core.GpuMat gpuMat6, @ByVal opencv_core.TermCriteria termCriteria);

    @Namespace("cv")
    @Name({"undistortPoints"})
    public static native void undistortPointsIter(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3, @ByVal opencv_core.Mat mat4, @ByVal opencv_core.Mat mat5, @ByVal opencv_core.Mat mat6, @ByVal opencv_core.TermCriteria termCriteria);

    @Namespace("cv")
    @Name({"undistortPoints"})
    public static native void undistortPointsIter(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3, @ByVal opencv_core.UMat uMat4, @ByVal opencv_core.UMat uMat5, @ByVal opencv_core.UMat uMat6, @ByVal opencv_core.TermCriteria termCriteria);

    @Namespace("cv")
    public static native void validateDisparity(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, int i2);

    @Namespace("cv")
    public static native void validateDisparity(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, int i2, int i3);

    @Namespace("cv")
    public static native void validateDisparity(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, int i2);

    @Namespace("cv")
    public static native void validateDisparity(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, int i2, int i3);

    @Namespace("cv")
    public static native void validateDisparity(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, int i2);

    @Namespace("cv")
    public static native void validateDisparity(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, int i2, int i3);

    static {
        Loader.load();
    }

    @NoOffset
    public static class CvLevMarq extends Pointer {
        public static final int CALC_J = 2;
        public static final int CHECK_ERR = 3;
        public static final int DONE = 0;
        public static final int STARTED = 1;

        private native void allocate();

        private native void allocate(int i, int i2);

        private native void allocate(int i, int i2, @ByVal(nullValue = "CvTermCriteria(cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,DBL_EPSILON))") opencv_core.CvTermCriteria cvTermCriteria, @Cast({"bool"}) boolean z);

        private native void allocateArray(long j);

        public native CvLevMarq J(opencv_core.CvMat cvMat);

        @opencv_core.Ptr
        public native opencv_core.CvMat J();

        public native CvLevMarq JtErr(opencv_core.CvMat cvMat);

        @opencv_core.Ptr
        public native opencv_core.CvMat JtErr();

        public native CvLevMarq JtJ(opencv_core.CvMat cvMat);

        @opencv_core.Ptr
        public native opencv_core.CvMat JtJ();

        public native CvLevMarq JtJN(opencv_core.CvMat cvMat);

        @opencv_core.Ptr
        public native opencv_core.CvMat JtJN();

        public native CvLevMarq JtJV(opencv_core.CvMat cvMat);

        @opencv_core.Ptr
        public native opencv_core.CvMat JtJV();

        public native CvLevMarq JtJW(opencv_core.CvMat cvMat);

        @opencv_core.Ptr
        public native opencv_core.CvMat JtJW();

        public native void clear();

        public native CvLevMarq completeSymmFlag(boolean z);

        @Cast({"bool"})
        public native boolean completeSymmFlag();

        public native CvLevMarq criteria(opencv_core.CvTermCriteria cvTermCriteria);

        @ByRef
        public native opencv_core.CvTermCriteria criteria();

        public native CvLevMarq err(opencv_core.CvMat cvMat);

        @opencv_core.Ptr
        public native opencv_core.CvMat err();

        public native double errNorm();

        public native CvLevMarq errNorm(double d);

        public native void init(int i, int i2);

        public native void init(int i, int i2, @ByVal(nullValue = "CvTermCriteria(cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,DBL_EPSILON))") opencv_core.CvTermCriteria cvTermCriteria, @Cast({"bool"}) boolean z);

        public native int iters();

        public native CvLevMarq iters(int i);

        public native int lambdaLg10();

        public native CvLevMarq lambdaLg10(int i);

        public native CvLevMarq mask(opencv_core.CvMat cvMat);

        @opencv_core.Ptr
        public native opencv_core.CvMat mask();

        public native CvLevMarq param(opencv_core.CvMat cvMat);

        @opencv_core.Ptr
        public native opencv_core.CvMat param();

        public native double prevErrNorm();

        public native CvLevMarq prevErrNorm(double d);

        public native CvLevMarq prevParam(opencv_core.CvMat cvMat);

        @opencv_core.Ptr
        public native opencv_core.CvMat prevParam();

        public native int solveMethod();

        public native CvLevMarq solveMethod(int i);

        public native int state();

        public native CvLevMarq state(int i);

        public native void step();

        @Cast({"bool"})
        public native boolean update(@ByPtrRef @Const opencv_core.CvMat cvMat, @ByPtrRef opencv_core.CvMat cvMat2, @ByPtrRef opencv_core.CvMat cvMat3);

        @Cast({"bool"})
        public native boolean updateAlt(@ByPtrRef @Const opencv_core.CvMat cvMat, @ByPtrRef opencv_core.CvMat cvMat2, @ByPtrRef opencv_core.CvMat cvMat3, @ByPtrRef DoubleBuffer doubleBuffer);

        @Cast({"bool"})
        public native boolean updateAlt(@ByPtrRef @Const opencv_core.CvMat cvMat, @ByPtrRef opencv_core.CvMat cvMat2, @ByPtrRef opencv_core.CvMat cvMat3, @ByPtrRef DoublePointer doublePointer);

        @Cast({"bool"})
        public native boolean updateAlt(@ByPtrRef @Const opencv_core.CvMat cvMat, @ByPtrRef opencv_core.CvMat cvMat2, @ByPtrRef opencv_core.CvMat cvMat3, @ByPtrRef double[] dArr);

        static {
            Loader.load();
        }

        public CvLevMarq(Pointer p) {
            super(p);
        }

        public CvLevMarq(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CvLevMarq position(long position) {
            return (CvLevMarq) super.position(position);
        }

        public CvLevMarq() {
            super((Pointer) null);
            allocate();
        }

        public CvLevMarq(int nparams, int nerrs, @ByVal(nullValue = "CvTermCriteria(cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,DBL_EPSILON))") opencv_core.CvTermCriteria criteria, @Cast({"bool"}) boolean completeSymmFlag) {
            super((Pointer) null);
            allocate(nparams, nerrs, criteria, completeSymmFlag);
        }

        public CvLevMarq(int nparams, int nerrs) {
            super((Pointer) null);
            allocate(nparams, nerrs);
        }
    }

    @Namespace("cv")
    public static class LMSolver extends opencv_core.Algorithm {
        @opencv_core.Ptr
        public static native LMSolver create(@opencv_core.Ptr Callback callback, int i);

        public native int getMaxIters();

        public native int run(@ByVal opencv_core.GpuMat gpuMat);

        public native int run(@ByVal opencv_core.Mat mat);

        public native int run(@ByVal opencv_core.UMat uMat);

        public native void setMaxIters(int i);

        static {
            Loader.load();
        }

        public LMSolver(Pointer p) {
            super(p);
        }

        public static class Callback extends Pointer {
            @Cast({"bool"})
            public native boolean compute(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

            @Cast({"bool"})
            public native boolean compute(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

            @Cast({"bool"})
            public native boolean compute(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

            static {
                Loader.load();
            }

            public Callback(Pointer p) {
                super(p);
            }
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class CirclesGridFinderParameters extends Pointer {
        public static final int ASYMMETRIC_GRID = 1;
        public static final int SYMMETRIC_GRID = 0;

        private native void allocate();

        private native void allocateArray(long j);

        public native float convexHullFactor();

        public native CirclesGridFinderParameters convexHullFactor(float f);

        public native CirclesGridFinderParameters densityNeighborhoodSize(opencv_core.Size2f size2f);

        @ByRef
        public native opencv_core.Size2f densityNeighborhoodSize();

        public native float edgeGain();

        public native CirclesGridFinderParameters edgeGain(float f);

        public native float edgePenalty();

        public native CirclesGridFinderParameters edgePenalty(float f);

        public native float existingVertexGain();

        public native CirclesGridFinderParameters existingVertexGain(float f);

        @Cast({"cv::CirclesGridFinderParameters::GridType"})
        public native int gridType();

        public native CirclesGridFinderParameters gridType(int i);

        public native int keypointScale();

        public native CirclesGridFinderParameters keypointScale(int i);

        public native int kmeansAttempts();

        public native CirclesGridFinderParameters kmeansAttempts(int i);

        public native float maxRectifiedDistance();

        public native CirclesGridFinderParameters maxRectifiedDistance(float f);

        public native float minDensity();

        public native CirclesGridFinderParameters minDensity(float f);

        public native int minDistanceToAddKeypoint();

        public native CirclesGridFinderParameters minDistanceToAddKeypoint(int i);

        public native float minGraphConfidence();

        public native CirclesGridFinderParameters minGraphConfidence(float f);

        public native float minRNGEdgeSwitchDist();

        public native CirclesGridFinderParameters minRNGEdgeSwitchDist(float f);

        public native float squareSize();

        public native CirclesGridFinderParameters squareSize(float f);

        public native float vertexGain();

        public native CirclesGridFinderParameters vertexGain(float f);

        public native float vertexPenalty();

        public native CirclesGridFinderParameters vertexPenalty(float f);

        static {
            Loader.load();
        }

        public CirclesGridFinderParameters(Pointer p) {
            super(p);
        }

        public CirclesGridFinderParameters(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public CirclesGridFinderParameters position(long position) {
            return (CirclesGridFinderParameters) super.position(position);
        }

        public CirclesGridFinderParameters() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv")
    public static class StereoMatcher extends opencv_core.Algorithm {
        public static final int DISP_SCALE = 16;
        public static final int DISP_SHIFT = 4;

        public native void compute(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

        public native void compute(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

        public native void compute(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

        public native int getBlockSize();

        public native int getDisp12MaxDiff();

        public native int getMinDisparity();

        public native int getNumDisparities();

        public native int getSpeckleRange();

        public native int getSpeckleWindowSize();

        public native void setBlockSize(int i);

        public native void setDisp12MaxDiff(int i);

        public native void setMinDisparity(int i);

        public native void setNumDisparities(int i);

        public native void setSpeckleRange(int i);

        public native void setSpeckleWindowSize(int i);

        static {
            Loader.load();
        }

        public StereoMatcher(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class StereoBM extends StereoMatcher {
        public static final int PREFILTER_NORMALIZED_RESPONSE = 0;
        public static final int PREFILTER_XSOBEL = 1;

        @opencv_core.Ptr
        public static native StereoBM create();

        @opencv_core.Ptr
        public static native StereoBM create(int i, int i2);

        public native int getPreFilterCap();

        public native int getPreFilterSize();

        public native int getPreFilterType();

        @ByVal
        public native opencv_core.Rect getROI1();

        @ByVal
        public native opencv_core.Rect getROI2();

        public native int getSmallerBlockSize();

        public native int getTextureThreshold();

        public native int getUniquenessRatio();

        public native void setPreFilterCap(int i);

        public native void setPreFilterSize(int i);

        public native void setPreFilterType(int i);

        public native void setROI1(@ByVal opencv_core.Rect rect);

        public native void setROI2(@ByVal opencv_core.Rect rect);

        public native void setSmallerBlockSize(int i);

        public native void setTextureThreshold(int i);

        public native void setUniquenessRatio(int i);

        static {
            Loader.load();
        }

        public StereoBM(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class StereoSGBM extends StereoMatcher {
        public static final int MODE_HH = 1;
        public static final int MODE_HH4 = 3;
        public static final int MODE_SGBM = 0;
        public static final int MODE_SGBM_3WAY = 2;

        @opencv_core.Ptr
        public static native StereoSGBM create();

        @opencv_core.Ptr
        public static native StereoSGBM create(int i, int i2, int i3, int i4, int i5, int i6, int i7, int i8, int i9, int i10, int i11);

        public native int getMode();

        public native int getP1();

        public native int getP2();

        public native int getPreFilterCap();

        public native int getUniquenessRatio();

        public native void setMode(int i);

        public native void setP1(int i);

        public native void setP2(int i);

        public native void setPreFilterCap(int i);

        public native void setUniquenessRatio(int i);

        static {
            Loader.load();
        }

        public StereoSGBM(Pointer p) {
            super(p);
        }
    }
}
