package org.bytedeco.javacpp;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Const;
import org.bytedeco.javacpp.annotation.MemberGetter;
import org.bytedeco.javacpp.annotation.Name;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.annotation.NoOffset;
import org.bytedeco.javacpp.annotation.StdVector;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.opencv_flann;
import org.bytedeco.javacpp.presets.opencv_core;

public class opencv_features2d extends org.bytedeco.javacpp.presets.opencv_features2d {
    public static final int DEFAULT = 0;
    public static final int DRAW_OVER_OUTIMG = 1;
    public static final int DRAW_RICH_KEYPOINTS = 4;
    public static final int NOT_DRAW_SINGLE_POINTS = 2;

    @Namespace("cv")
    public static native void AGAST(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.KeyPointVector keyPointVector, int i);

    @Namespace("cv")
    public static native void AGAST(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.KeyPointVector keyPointVector, int i, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void AGAST(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.KeyPointVector keyPointVector, int i, @Cast({"bool"}) boolean z, @Cast({"cv::AgastFeatureDetector::DetectorType"}) int i2);

    @Namespace("cv")
    public static native void AGAST(@ByVal opencv_core.Mat mat, @ByRef opencv_core.KeyPointVector keyPointVector, int i);

    @Namespace("cv")
    public static native void AGAST(@ByVal opencv_core.Mat mat, @ByRef opencv_core.KeyPointVector keyPointVector, int i, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void AGAST(@ByVal opencv_core.Mat mat, @ByRef opencv_core.KeyPointVector keyPointVector, int i, @Cast({"bool"}) boolean z, @Cast({"cv::AgastFeatureDetector::DetectorType"}) int i2);

    @Namespace("cv")
    public static native void AGAST(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.KeyPointVector keyPointVector, int i);

    @Namespace("cv")
    public static native void AGAST(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.KeyPointVector keyPointVector, int i, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void AGAST(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.KeyPointVector keyPointVector, int i, @Cast({"bool"}) boolean z, @Cast({"cv::AgastFeatureDetector::DetectorType"}) int i2);

    @Namespace("cv")
    public static native void FAST(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.KeyPointVector keyPointVector, int i);

    @Namespace("cv")
    public static native void FAST(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.KeyPointVector keyPointVector, int i, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void FAST(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.KeyPointVector keyPointVector, int i, @Cast({"bool"}) boolean z, @Cast({"cv::FastFeatureDetector::DetectorType"}) int i2);

    @Namespace("cv")
    public static native void FAST(@ByVal opencv_core.Mat mat, @ByRef opencv_core.KeyPointVector keyPointVector, int i);

    @Namespace("cv")
    public static native void FAST(@ByVal opencv_core.Mat mat, @ByRef opencv_core.KeyPointVector keyPointVector, int i, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void FAST(@ByVal opencv_core.Mat mat, @ByRef opencv_core.KeyPointVector keyPointVector, int i, @Cast({"bool"}) boolean z, @Cast({"cv::FastFeatureDetector::DetectorType"}) int i2);

    @Namespace("cv")
    public static native void FAST(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.KeyPointVector keyPointVector, int i);

    @Namespace("cv")
    public static native void FAST(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.KeyPointVector keyPointVector, int i, @Cast({"bool"}) boolean z);

    @Namespace("cv")
    public static native void FAST(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.KeyPointVector keyPointVector, int i, @Cast({"bool"}) boolean z, @Cast({"cv::FastFeatureDetector::DetectorType"}) int i2);

    @Namespace("cv")
    @Cast({"cv::DrawMatchesFlags"})
    @Name({"operator &"})
    public static native int and(@Cast({"const cv::DrawMatchesFlags"}) int i, @Cast({"const cv::DrawMatchesFlags"}) int i2);

    @ByRef
    @Cast({"cv::DrawMatchesFlags*"})
    @Name({"operator &="})
    @Namespace("cv")
    public static native IntBuffer andPut(@ByRef @Cast({"cv::DrawMatchesFlags*"}) IntBuffer intBuffer, @Cast({"const cv::DrawMatchesFlags"}) int i);

    @ByRef
    @Cast({"cv::DrawMatchesFlags*"})
    @Name({"operator &="})
    @Namespace("cv")
    public static native IntPointer andPut(@ByRef @Cast({"cv::DrawMatchesFlags*"}) IntPointer intPointer, @Cast({"const cv::DrawMatchesFlags"}) int i);

    @ByRef
    @Cast({"cv::DrawMatchesFlags*"})
    @Name({"operator &="})
    @Namespace("cv")
    public static native int[] andPut(@ByRef @Cast({"cv::DrawMatchesFlags*"}) int[] iArr, @Cast({"const cv::DrawMatchesFlags"}) int i);

    @Namespace("cv")
    public static native void computeRecallPrecisionCurve(@ByRef @Const opencv_core.DMatchVectorVector dMatchVectorVector, @ByRef @Cast({"const std::vector<std::vector<uchar> >*"}) opencv_core.ByteVectorVector byteVectorVector, @ByRef opencv_core.Point2fVector point2fVector);

    @Namespace("cv")
    public static native void drawKeypoints(@ByVal opencv_core.GpuMat gpuMat, @ByRef @Const opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv")
    public static native void drawKeypoints(@ByVal opencv_core.GpuMat gpuMat, @ByRef @Const opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::Scalar::all(-1)") @Const opencv_core.Scalar scalar, @Cast({"cv::DrawMatchesFlags"}) int i);

    @Namespace("cv")
    public static native void drawKeypoints(@ByVal opencv_core.Mat mat, @ByRef @Const opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.Mat mat2);

    @Namespace("cv")
    public static native void drawKeypoints(@ByVal opencv_core.Mat mat, @ByRef @Const opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::Scalar::all(-1)") @Const opencv_core.Scalar scalar, @Cast({"cv::DrawMatchesFlags"}) int i);

    @Namespace("cv")
    public static native void drawKeypoints(@ByVal opencv_core.UMat uMat, @ByRef @Const opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv")
    public static native void drawKeypoints(@ByVal opencv_core.UMat uMat, @ByRef @Const opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::Scalar::all(-1)") @Const opencv_core.Scalar scalar, @Cast({"cv::DrawMatchesFlags"}) int i);

    @Namespace("cv")
    public static native void drawMatches(@ByVal opencv_core.GpuMat gpuMat, @ByRef @Const opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.GpuMat gpuMat2, @ByRef @Const opencv_core.KeyPointVector keyPointVector2, @ByRef @Const opencv_core.DMatchVector dMatchVector, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv")
    public static native void drawMatches(@ByVal opencv_core.GpuMat gpuMat, @ByRef @Const opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.GpuMat gpuMat2, @ByRef @Const opencv_core.KeyPointVector keyPointVector2, @ByRef @Const opencv_core.DMatchVector dMatchVector, @ByVal opencv_core.GpuMat gpuMat3, @ByRef(nullValue = "cv::Scalar::all(-1)") @Const opencv_core.Scalar scalar, @ByRef(nullValue = "cv::Scalar::all(-1)") @Const opencv_core.Scalar scalar2, @Cast({"char*"}) @StdVector ByteBuffer byteBuffer, @Cast({"cv::DrawMatchesFlags"}) int i);

    @Namespace("cv")
    public static native void drawMatches(@ByVal opencv_core.GpuMat gpuMat, @ByRef @Const opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.GpuMat gpuMat2, @ByRef @Const opencv_core.KeyPointVector keyPointVector2, @ByRef @Const opencv_core.DMatchVector dMatchVector, @ByVal opencv_core.GpuMat gpuMat3, @ByRef(nullValue = "cv::Scalar::all(-1)") @Const opencv_core.Scalar scalar, @ByRef(nullValue = "cv::Scalar::all(-1)") @Const opencv_core.Scalar scalar2, @Cast({"char*"}) @StdVector BytePointer bytePointer, @Cast({"cv::DrawMatchesFlags"}) int i);

    @Namespace("cv")
    public static native void drawMatches(@ByVal opencv_core.GpuMat gpuMat, @ByRef @Const opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.GpuMat gpuMat2, @ByRef @Const opencv_core.KeyPointVector keyPointVector2, @ByRef @Const opencv_core.DMatchVector dMatchVector, @ByVal opencv_core.GpuMat gpuMat3, @ByRef(nullValue = "cv::Scalar::all(-1)") @Const opencv_core.Scalar scalar, @ByRef(nullValue = "cv::Scalar::all(-1)") @Const opencv_core.Scalar scalar2, @Cast({"char*"}) @StdVector byte[] bArr, @Cast({"cv::DrawMatchesFlags"}) int i);

    @Namespace("cv")
    public static native void drawMatches(@ByVal opencv_core.Mat mat, @ByRef @Const opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.Mat mat2, @ByRef @Const opencv_core.KeyPointVector keyPointVector2, @ByRef @Const opencv_core.DMatchVector dMatchVector, @ByVal opencv_core.Mat mat3);

    @Namespace("cv")
    public static native void drawMatches(@ByVal opencv_core.Mat mat, @ByRef @Const opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.Mat mat2, @ByRef @Const opencv_core.KeyPointVector keyPointVector2, @ByRef @Const opencv_core.DMatchVector dMatchVector, @ByVal opencv_core.Mat mat3, @ByRef(nullValue = "cv::Scalar::all(-1)") @Const opencv_core.Scalar scalar, @ByRef(nullValue = "cv::Scalar::all(-1)") @Const opencv_core.Scalar scalar2, @Cast({"char*"}) @StdVector ByteBuffer byteBuffer, @Cast({"cv::DrawMatchesFlags"}) int i);

    @Namespace("cv")
    public static native void drawMatches(@ByVal opencv_core.Mat mat, @ByRef @Const opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.Mat mat2, @ByRef @Const opencv_core.KeyPointVector keyPointVector2, @ByRef @Const opencv_core.DMatchVector dMatchVector, @ByVal opencv_core.Mat mat3, @ByRef(nullValue = "cv::Scalar::all(-1)") @Const opencv_core.Scalar scalar, @ByRef(nullValue = "cv::Scalar::all(-1)") @Const opencv_core.Scalar scalar2, @Cast({"char*"}) @StdVector BytePointer bytePointer, @Cast({"cv::DrawMatchesFlags"}) int i);

    @Namespace("cv")
    public static native void drawMatches(@ByVal opencv_core.Mat mat, @ByRef @Const opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.Mat mat2, @ByRef @Const opencv_core.KeyPointVector keyPointVector2, @ByRef @Const opencv_core.DMatchVector dMatchVector, @ByVal opencv_core.Mat mat3, @ByRef(nullValue = "cv::Scalar::all(-1)") @Const opencv_core.Scalar scalar, @ByRef(nullValue = "cv::Scalar::all(-1)") @Const opencv_core.Scalar scalar2, @Cast({"char*"}) @StdVector byte[] bArr, @Cast({"cv::DrawMatchesFlags"}) int i);

    @Namespace("cv")
    public static native void drawMatches(@ByVal opencv_core.UMat uMat, @ByRef @Const opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.UMat uMat2, @ByRef @Const opencv_core.KeyPointVector keyPointVector2, @ByRef @Const opencv_core.DMatchVector dMatchVector, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv")
    public static native void drawMatches(@ByVal opencv_core.UMat uMat, @ByRef @Const opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.UMat uMat2, @ByRef @Const opencv_core.KeyPointVector keyPointVector2, @ByRef @Const opencv_core.DMatchVector dMatchVector, @ByVal opencv_core.UMat uMat3, @ByRef(nullValue = "cv::Scalar::all(-1)") @Const opencv_core.Scalar scalar, @ByRef(nullValue = "cv::Scalar::all(-1)") @Const opencv_core.Scalar scalar2, @Cast({"char*"}) @StdVector ByteBuffer byteBuffer, @Cast({"cv::DrawMatchesFlags"}) int i);

    @Namespace("cv")
    public static native void drawMatches(@ByVal opencv_core.UMat uMat, @ByRef @Const opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.UMat uMat2, @ByRef @Const opencv_core.KeyPointVector keyPointVector2, @ByRef @Const opencv_core.DMatchVector dMatchVector, @ByVal opencv_core.UMat uMat3, @ByRef(nullValue = "cv::Scalar::all(-1)") @Const opencv_core.Scalar scalar, @ByRef(nullValue = "cv::Scalar::all(-1)") @Const opencv_core.Scalar scalar2, @Cast({"char*"}) @StdVector BytePointer bytePointer, @Cast({"cv::DrawMatchesFlags"}) int i);

    @Namespace("cv")
    public static native void drawMatches(@ByVal opencv_core.UMat uMat, @ByRef @Const opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.UMat uMat2, @ByRef @Const opencv_core.KeyPointVector keyPointVector2, @ByRef @Const opencv_core.DMatchVector dMatchVector, @ByVal opencv_core.UMat uMat3, @ByRef(nullValue = "cv::Scalar::all(-1)") @Const opencv_core.Scalar scalar, @ByRef(nullValue = "cv::Scalar::all(-1)") @Const opencv_core.Scalar scalar2, @Cast({"char*"}) @StdVector byte[] bArr, @Cast({"cv::DrawMatchesFlags"}) int i);

    @Namespace("cv")
    @Name({"drawMatches"})
    public static native void drawMatchesKnn(@ByVal opencv_core.GpuMat gpuMat, @ByRef @Const opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.GpuMat gpuMat2, @ByRef @Const opencv_core.KeyPointVector keyPointVector2, @ByRef @Const opencv_core.DMatchVectorVector dMatchVectorVector, @ByVal opencv_core.GpuMat gpuMat3);

    @Namespace("cv")
    @Name({"drawMatches"})
    public static native void drawMatchesKnn(@ByVal opencv_core.GpuMat gpuMat, @ByRef @Const opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.GpuMat gpuMat2, @ByRef @Const opencv_core.KeyPointVector keyPointVector2, @ByRef @Const opencv_core.DMatchVectorVector dMatchVectorVector, @ByVal opencv_core.GpuMat gpuMat3, @ByRef(nullValue = "cv::Scalar::all(-1)") @Const opencv_core.Scalar scalar, @ByRef(nullValue = "cv::Scalar::all(-1)") @Const opencv_core.Scalar scalar2, @ByRef(nullValue = "std::vector<std::vector<char> >()") @Cast({"const std::vector<std::vector<char> >*"}) opencv_core.ByteVectorVector byteVectorVector, @Cast({"cv::DrawMatchesFlags"}) int i);

    @Namespace("cv")
    @Name({"drawMatches"})
    public static native void drawMatchesKnn(@ByVal opencv_core.Mat mat, @ByRef @Const opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.Mat mat2, @ByRef @Const opencv_core.KeyPointVector keyPointVector2, @ByRef @Const opencv_core.DMatchVectorVector dMatchVectorVector, @ByVal opencv_core.Mat mat3);

    @Namespace("cv")
    @Name({"drawMatches"})
    public static native void drawMatchesKnn(@ByVal opencv_core.Mat mat, @ByRef @Const opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.Mat mat2, @ByRef @Const opencv_core.KeyPointVector keyPointVector2, @ByRef @Const opencv_core.DMatchVectorVector dMatchVectorVector, @ByVal opencv_core.Mat mat3, @ByRef(nullValue = "cv::Scalar::all(-1)") @Const opencv_core.Scalar scalar, @ByRef(nullValue = "cv::Scalar::all(-1)") @Const opencv_core.Scalar scalar2, @ByRef(nullValue = "std::vector<std::vector<char> >()") @Cast({"const std::vector<std::vector<char> >*"}) opencv_core.ByteVectorVector byteVectorVector, @Cast({"cv::DrawMatchesFlags"}) int i);

    @Namespace("cv")
    @Name({"drawMatches"})
    public static native void drawMatchesKnn(@ByVal opencv_core.UMat uMat, @ByRef @Const opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.UMat uMat2, @ByRef @Const opencv_core.KeyPointVector keyPointVector2, @ByRef @Const opencv_core.DMatchVectorVector dMatchVectorVector, @ByVal opencv_core.UMat uMat3);

    @Namespace("cv")
    @Name({"drawMatches"})
    public static native void drawMatchesKnn(@ByVal opencv_core.UMat uMat, @ByRef @Const opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.UMat uMat2, @ByRef @Const opencv_core.KeyPointVector keyPointVector2, @ByRef @Const opencv_core.DMatchVectorVector dMatchVectorVector, @ByVal opencv_core.UMat uMat3, @ByRef(nullValue = "cv::Scalar::all(-1)") @Const opencv_core.Scalar scalar, @ByRef(nullValue = "cv::Scalar::all(-1)") @Const opencv_core.Scalar scalar2, @ByRef(nullValue = "std::vector<std::vector<char> >()") @Cast({"const std::vector<std::vector<char> >*"}) opencv_core.ByteVectorVector byteVectorVector, @Cast({"cv::DrawMatchesFlags"}) int i);

    @Namespace("cv")
    @Cast({"bool"})
    @Name({"operator =="})
    public static native boolean equals(@Cast({"const cv::DrawMatchesFlags"}) int i, int i2);

    @Namespace("cv")
    public static native void evaluateFeatureDetector(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2, @ByRef @Const opencv_core.Mat mat3, opencv_core.KeyPointVector keyPointVector, opencv_core.KeyPointVector keyPointVector2, @ByRef FloatBuffer floatBuffer, @ByRef IntBuffer intBuffer);

    @Namespace("cv")
    public static native void evaluateFeatureDetector(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2, @ByRef @Const opencv_core.Mat mat3, opencv_core.KeyPointVector keyPointVector, opencv_core.KeyPointVector keyPointVector2, @ByRef FloatBuffer floatBuffer, @ByRef IntBuffer intBuffer, @opencv_core.Ptr @Cast({"cv::FeatureDetector*"}) Feature2D feature2D);

    @Namespace("cv")
    public static native void evaluateFeatureDetector(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2, @ByRef @Const opencv_core.Mat mat3, opencv_core.KeyPointVector keyPointVector, opencv_core.KeyPointVector keyPointVector2, @ByRef FloatPointer floatPointer, @ByRef IntPointer intPointer);

    @Namespace("cv")
    public static native void evaluateFeatureDetector(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2, @ByRef @Const opencv_core.Mat mat3, opencv_core.KeyPointVector keyPointVector, opencv_core.KeyPointVector keyPointVector2, @ByRef FloatPointer floatPointer, @ByRef IntPointer intPointer, @opencv_core.Ptr @Cast({"cv::FeatureDetector*"}) Feature2D feature2D);

    @Namespace("cv")
    public static native void evaluateFeatureDetector(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2, @ByRef @Const opencv_core.Mat mat3, opencv_core.KeyPointVector keyPointVector, opencv_core.KeyPointVector keyPointVector2, @ByRef float[] fArr, @ByRef int[] iArr);

    @Namespace("cv")
    public static native void evaluateFeatureDetector(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2, @ByRef @Const opencv_core.Mat mat3, opencv_core.KeyPointVector keyPointVector, opencv_core.KeyPointVector keyPointVector2, @ByRef float[] fArr, @ByRef int[] iArr, @opencv_core.Ptr @Cast({"cv::FeatureDetector*"}) Feature2D feature2D);

    @Namespace("cv")
    public static native int getNearestPoint(@ByRef @Const opencv_core.Point2fVector point2fVector, float f);

    @Namespace("cv")
    public static native float getRecall(@ByRef @Const opencv_core.Point2fVector point2fVector, float f);

    @Namespace("cv")
    @Cast({"bool"})
    @Name({"operator !"})
    public static native boolean not(@Cast({"const cv::DrawMatchesFlags"}) int i);

    @Namespace("cv")
    @Cast({"bool"})
    @Name({"operator !="})
    public static native boolean notEquals(@Cast({"const cv::DrawMatchesFlags"}) int i, int i2);

    @Namespace("cv")
    @Cast({"cv::DrawMatchesFlags"})
    @Name({"operator |"})
    public static native int or(@Cast({"const cv::DrawMatchesFlags"}) int i, @Cast({"const cv::DrawMatchesFlags"}) int i2);

    @ByRef
    @Cast({"cv::DrawMatchesFlags*"})
    @Name({"operator |="})
    @Namespace("cv")
    public static native IntBuffer orPut(@ByRef @Cast({"cv::DrawMatchesFlags*"}) IntBuffer intBuffer, @Cast({"const cv::DrawMatchesFlags"}) int i);

    @ByRef
    @Cast({"cv::DrawMatchesFlags*"})
    @Name({"operator |="})
    @Namespace("cv")
    public static native IntPointer orPut(@ByRef @Cast({"cv::DrawMatchesFlags*"}) IntPointer intPointer, @Cast({"const cv::DrawMatchesFlags"}) int i);

    @ByRef
    @Cast({"cv::DrawMatchesFlags*"})
    @Name({"operator |="})
    @Namespace("cv")
    public static native int[] orPut(@ByRef @Cast({"cv::DrawMatchesFlags*"}) int[] iArr, @Cast({"const cv::DrawMatchesFlags"}) int i);

    @Namespace("cv")
    @Cast({"cv::DrawMatchesFlags"})
    @Name({"operator ^"})
    public static native int xor(@Cast({"const cv::DrawMatchesFlags"}) int i, @Cast({"const cv::DrawMatchesFlags"}) int i2);

    @ByRef
    @Cast({"cv::DrawMatchesFlags*"})
    @Name({"operator ^="})
    @Namespace("cv")
    public static native IntBuffer xorPut(@ByRef @Cast({"cv::DrawMatchesFlags*"}) IntBuffer intBuffer, @Cast({"const cv::DrawMatchesFlags"}) int i);

    @ByRef
    @Cast({"cv::DrawMatchesFlags*"})
    @Name({"operator ^="})
    @Namespace("cv")
    public static native IntPointer xorPut(@ByRef @Cast({"cv::DrawMatchesFlags*"}) IntPointer intPointer, @Cast({"const cv::DrawMatchesFlags"}) int i);

    @ByRef
    @Cast({"cv::DrawMatchesFlags*"})
    @Name({"operator ^="})
    @Namespace("cv")
    public static native int[] xorPut(@ByRef @Cast({"cv::DrawMatchesFlags*"}) int[] iArr, @Cast({"const cv::DrawMatchesFlags"}) int i);

    static {
        Loader.load();
    }

    @Namespace("cv")
    public static class KeyPointsFilter extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        public static native void removeDuplicated(@ByRef opencv_core.KeyPointVector keyPointVector);

        public static native void removeDuplicatedSorted(@ByRef opencv_core.KeyPointVector keyPointVector);

        public static native void retainBest(@ByRef opencv_core.KeyPointVector keyPointVector, int i);

        public static native void runByImageBorder(@ByRef opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.Size size, int i);

        public static native void runByKeypointSize(@ByRef opencv_core.KeyPointVector keyPointVector, float f);

        public static native void runByKeypointSize(@ByRef opencv_core.KeyPointVector keyPointVector, float f, float f2);

        public static native void runByPixelsMask(@ByRef opencv_core.KeyPointVector keyPointVector, @ByRef @Const opencv_core.Mat mat);

        static {
            Loader.load();
        }

        public KeyPointsFilter(Pointer p) {
            super(p);
        }

        public KeyPointsFilter(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public KeyPointsFilter position(long position) {
            return (KeyPointsFilter) super.position(position);
        }

        public KeyPointsFilter() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv")
    public static class Feature2D extends opencv_core.Algorithm {
        private native void allocate();

        private native void allocateArray(long j);

        public native void compute(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.GpuMat gpuMat2);

        public native void compute(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByRef opencv_core.KeyPointVectorVector keyPointVectorVector, @ByVal opencv_core.GpuMatVector gpuMatVector2);

        public native void compute(@ByVal opencv_core.Mat mat, @ByRef opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.Mat mat2);

        public native void compute(@ByVal opencv_core.MatVector matVector, @ByRef opencv_core.KeyPointVectorVector keyPointVectorVector, @ByVal opencv_core.MatVector matVector2);

        public native void compute(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.UMat uMat2);

        public native void compute(@ByVal opencv_core.UMatVector uMatVector, @ByRef opencv_core.KeyPointVectorVector keyPointVectorVector, @ByVal opencv_core.UMatVector uMatVector2);

        public native int defaultNorm();

        public native int descriptorSize();

        public native int descriptorType();

        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.KeyPointVector keyPointVector);

        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.KeyPointVector keyPointVector, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat2);

        public native void detect(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByRef opencv_core.KeyPointVectorVector keyPointVectorVector);

        public native void detect(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByRef opencv_core.KeyPointVectorVector keyPointVectorVector, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector2);

        public native void detect(@ByVal opencv_core.Mat mat, @ByRef opencv_core.KeyPointVector keyPointVector);

        public native void detect(@ByVal opencv_core.Mat mat, @ByRef opencv_core.KeyPointVector keyPointVector, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat2);

        public native void detect(@ByVal opencv_core.MatVector matVector, @ByRef opencv_core.KeyPointVectorVector keyPointVectorVector);

        public native void detect(@ByVal opencv_core.MatVector matVector, @ByRef opencv_core.KeyPointVectorVector keyPointVectorVector, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector2);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.KeyPointVector keyPointVector);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.KeyPointVector keyPointVector, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat2);

        public native void detect(@ByVal opencv_core.UMatVector uMatVector, @ByRef opencv_core.KeyPointVectorVector keyPointVectorVector);

        public native void detect(@ByVal opencv_core.UMatVector uMatVector, @ByRef opencv_core.KeyPointVectorVector keyPointVectorVector, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector2);

        public native void detectAndCompute(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.GpuMat gpuMat3);

        public native void detectAndCompute(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.GpuMat gpuMat3, @Cast({"bool"}) boolean z);

        public native void detectAndCompute(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.Mat mat3);

        public native void detectAndCompute(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.Mat mat3, @Cast({"bool"}) boolean z);

        public native void detectAndCompute(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.UMat uMat3);

        public native void detectAndCompute(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.UMat uMat3, @Cast({"bool"}) boolean z);

        @Cast({"bool"})
        public native boolean empty();

        @opencv_core.Str
        public native BytePointer getDefaultName();

        public native void read(@opencv_core.Str String str);

        public native void read(@opencv_core.Str BytePointer bytePointer);

        public native void read(@ByRef @Const opencv_core.FileNode fileNode);

        public native void write(@opencv_core.Str String str);

        public native void write(@opencv_core.Str BytePointer bytePointer);

        public native void write(@ByRef opencv_core.FileStorage fileStorage);

        public native void write(@opencv_core.Ptr opencv_core.FileStorage fileStorage, @opencv_core.Str String str);

        public native void write(@opencv_core.Ptr opencv_core.FileStorage fileStorage, @opencv_core.Str BytePointer bytePointer);

        static {
            Loader.load();
        }

        public Feature2D() {
            super((Pointer) null);
            allocate();
        }

        public Feature2D(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Feature2D(Pointer p) {
            super(p);
        }

        public Feature2D position(long position) {
            return (Feature2D) super.position(position);
        }
    }

    @Namespace("cv")
    public static class BRISK extends Feature2D {
        private native void allocate();

        private native void allocateArray(long j);

        @opencv_core.Ptr
        public static native BRISK create();

        @opencv_core.Ptr
        public static native BRISK create(int i, int i2, float f);

        @opencv_core.Ptr
        public static native BRISK create(int i, int i2, @StdVector FloatBuffer floatBuffer, @StdVector IntBuffer intBuffer);

        @opencv_core.Ptr
        public static native BRISK create(int i, int i2, @StdVector FloatBuffer floatBuffer, @StdVector IntBuffer intBuffer, float f, float f2, @StdVector IntBuffer intBuffer2);

        @opencv_core.Ptr
        public static native BRISK create(int i, int i2, @StdVector FloatPointer floatPointer, @StdVector IntPointer intPointer);

        @opencv_core.Ptr
        public static native BRISK create(int i, int i2, @StdVector FloatPointer floatPointer, @StdVector IntPointer intPointer, float f, float f2, @StdVector IntPointer intPointer2);

        @opencv_core.Ptr
        public static native BRISK create(int i, int i2, @StdVector float[] fArr, @StdVector int[] iArr);

        @opencv_core.Ptr
        public static native BRISK create(int i, int i2, @StdVector float[] fArr, @StdVector int[] iArr, float f, float f2, @StdVector int[] iArr2);

        @opencv_core.Ptr
        public static native BRISK create(@StdVector FloatBuffer floatBuffer, @StdVector IntBuffer intBuffer);

        @opencv_core.Ptr
        public static native BRISK create(@StdVector FloatBuffer floatBuffer, @StdVector IntBuffer intBuffer, float f, float f2, @StdVector IntBuffer intBuffer2);

        @opencv_core.Ptr
        public static native BRISK create(@StdVector FloatPointer floatPointer, @StdVector IntPointer intPointer);

        @opencv_core.Ptr
        public static native BRISK create(@StdVector FloatPointer floatPointer, @StdVector IntPointer intPointer, float f, float f2, @StdVector IntPointer intPointer2);

        @opencv_core.Ptr
        public static native BRISK create(@StdVector float[] fArr, @StdVector int[] iArr);

        @opencv_core.Ptr
        public static native BRISK create(@StdVector float[] fArr, @StdVector int[] iArr, float f, float f2, @StdVector int[] iArr2);

        @opencv_core.Str
        public native BytePointer getDefaultName();

        public native int getOctaves();

        public native int getThreshold();

        public native void setOctaves(int i);

        public native void setThreshold(int i);

        static {
            Loader.load();
        }

        public BRISK() {
            super((Pointer) null);
            allocate();
        }

        public BRISK(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public BRISK(Pointer p) {
            super(p);
        }

        public BRISK position(long position) {
            return (BRISK) super.position(position);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class ORB extends Feature2D {
        public static final int FAST_SCORE = 1;
        public static final int HARRIS_SCORE = 0;
        public static final int kBytes = kBytes();

        @opencv_core.Ptr
        public static native ORB create();

        @opencv_core.Ptr
        public static native ORB create(int i, float f, int i2, int i3, int i4, int i5, @Cast({"cv::ORB::ScoreType"}) int i6, int i7, int i8);

        @MemberGetter
        public static native int kBytes();

        @opencv_core.Str
        public native BytePointer getDefaultName();

        public native int getEdgeThreshold();

        public native int getFastThreshold();

        public native int getFirstLevel();

        public native int getMaxFeatures();

        public native int getNLevels();

        public native int getPatchSize();

        public native double getScaleFactor();

        @Cast({"cv::ORB::ScoreType"})
        public native int getScoreType();

        public native int getWTA_K();

        public native void setEdgeThreshold(int i);

        public native void setFastThreshold(int i);

        public native void setFirstLevel(int i);

        public native void setMaxFeatures(int i);

        public native void setNLevels(int i);

        public native void setPatchSize(int i);

        public native void setScaleFactor(double d);

        public native void setScoreType(@Cast({"cv::ORB::ScoreType"}) int i);

        public native void setWTA_K(int i);

        static {
            Loader.load();
        }

        public ORB(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class MSER extends Feature2D {
        @opencv_core.Ptr
        public static native MSER create();

        @opencv_core.Ptr
        public static native MSER create(int i, int i2, int i3, double d, double d2, int i4, double d3, double d4, int i5);

        public native void detectRegions(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector);

        public native void detectRegions(@ByVal opencv_core.Mat mat, @ByRef opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector);

        public native void detectRegions(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector);

        @opencv_core.Str
        public native BytePointer getDefaultName();

        public native int getDelta();

        public native int getMaxArea();

        public native int getMinArea();

        @Cast({"bool"})
        public native boolean getPass2Only();

        public native void setDelta(int i);

        public native void setMaxArea(int i);

        public native void setMinArea(int i);

        public native void setPass2Only(@Cast({"bool"}) boolean z);

        static {
            Loader.load();
        }

        public MSER(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class FastFeatureDetector extends Feature2D {
        public static final int FAST_N = 10002;
        public static final int NONMAX_SUPPRESSION = 10001;
        public static final int THRESHOLD = 10000;
        public static final int TYPE_5_8 = 0;
        public static final int TYPE_7_12 = 1;
        public static final int TYPE_9_16 = 2;

        @opencv_core.Ptr
        public static native FastFeatureDetector create();

        @opencv_core.Ptr
        public static native FastFeatureDetector create(int i, @Cast({"bool"}) boolean z, @Cast({"cv::FastFeatureDetector::DetectorType"}) int i2);

        @opencv_core.Str
        public native BytePointer getDefaultName();

        @Cast({"bool"})
        public native boolean getNonmaxSuppression();

        public native int getThreshold();

        @Cast({"cv::FastFeatureDetector::DetectorType"})
        public native int getType();

        public native void setNonmaxSuppression(@Cast({"bool"}) boolean z);

        public native void setThreshold(int i);

        public native void setType(@Cast({"cv::FastFeatureDetector::DetectorType"}) int i);

        static {
            Loader.load();
        }

        public FastFeatureDetector(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class AgastFeatureDetector extends Feature2D {
        public static final int AGAST_5_8 = 0;
        public static final int AGAST_7_12d = 1;
        public static final int AGAST_7_12s = 2;
        public static final int NONMAX_SUPPRESSION = 10001;
        public static final int OAST_9_16 = 3;
        public static final int THRESHOLD = 10000;

        @opencv_core.Ptr
        public static native AgastFeatureDetector create();

        @opencv_core.Ptr
        public static native AgastFeatureDetector create(int i, @Cast({"bool"}) boolean z, @Cast({"cv::AgastFeatureDetector::DetectorType"}) int i2);

        @opencv_core.Str
        public native BytePointer getDefaultName();

        @Cast({"bool"})
        public native boolean getNonmaxSuppression();

        public native int getThreshold();

        @Cast({"cv::AgastFeatureDetector::DetectorType"})
        public native int getType();

        public native void setNonmaxSuppression(@Cast({"bool"}) boolean z);

        public native void setThreshold(int i);

        public native void setType(@Cast({"cv::AgastFeatureDetector::DetectorType"}) int i);

        static {
            Loader.load();
        }

        public AgastFeatureDetector(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class GFTTDetector extends Feature2D {
        @opencv_core.Ptr
        public static native GFTTDetector create();

        @opencv_core.Ptr
        public static native GFTTDetector create(int i, double d, double d2, int i2, int i3);

        @opencv_core.Ptr
        public static native GFTTDetector create(int i, double d, double d2, int i2, int i3, @Cast({"bool"}) boolean z, double d3);

        @opencv_core.Ptr
        public static native GFTTDetector create(int i, double d, double d2, int i2, @Cast({"bool"}) boolean z, double d3);

        public native int getBlockSize();

        @opencv_core.Str
        public native BytePointer getDefaultName();

        @Cast({"bool"})
        public native boolean getHarrisDetector();

        public native double getK();

        public native int getMaxFeatures();

        public native double getMinDistance();

        public native double getQualityLevel();

        public native void setBlockSize(int i);

        public native void setHarrisDetector(@Cast({"bool"}) boolean z);

        public native void setK(double d);

        public native void setMaxFeatures(int i);

        public native void setMinDistance(double d);

        public native void setQualityLevel(double d);

        static {
            Loader.load();
        }

        public GFTTDetector(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class SimpleBlobDetector extends Feature2D {
        private native void allocate();

        private native void allocateArray(long j);

        @opencv_core.Ptr
        public static native SimpleBlobDetector create();

        @opencv_core.Ptr
        public static native SimpleBlobDetector create(@ByRef(nullValue = "cv::SimpleBlobDetector::Params()") @Const Params params);

        @opencv_core.Str
        public native BytePointer getDefaultName();

        static {
            Loader.load();
        }

        public SimpleBlobDetector() {
            super((Pointer) null);
            allocate();
        }

        public SimpleBlobDetector(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public SimpleBlobDetector(Pointer p) {
            super(p);
        }

        public SimpleBlobDetector position(long position) {
            return (SimpleBlobDetector) super.position(position);
        }

        @NoOffset
        public static class Params extends Pointer {
            private native void allocate();

            private native void allocateArray(long j);

            @Cast({"uchar"})
            public native byte blobColor();

            public native Params blobColor(byte b);

            public native Params filterByArea(boolean z);

            @Cast({"bool"})
            public native boolean filterByArea();

            public native Params filterByCircularity(boolean z);

            @Cast({"bool"})
            public native boolean filterByCircularity();

            public native Params filterByColor(boolean z);

            @Cast({"bool"})
            public native boolean filterByColor();

            public native Params filterByConvexity(boolean z);

            @Cast({"bool"})
            public native boolean filterByConvexity();

            public native Params filterByInertia(boolean z);

            @Cast({"bool"})
            public native boolean filterByInertia();

            public native float maxArea();

            public native Params maxArea(float f);

            public native float maxCircularity();

            public native Params maxCircularity(float f);

            public native float maxConvexity();

            public native Params maxConvexity(float f);

            public native float maxInertiaRatio();

            public native Params maxInertiaRatio(float f);

            public native float maxThreshold();

            public native Params maxThreshold(float f);

            public native float minArea();

            public native Params minArea(float f);

            public native float minCircularity();

            public native Params minCircularity(float f);

            public native float minConvexity();

            public native Params minConvexity(float f);

            public native float minDistBetweenBlobs();

            public native Params minDistBetweenBlobs(float f);

            public native float minInertiaRatio();

            public native Params minInertiaRatio(float f);

            @Cast({"size_t"})
            public native long minRepeatability();

            public native Params minRepeatability(long j);

            public native float minThreshold();

            public native Params minThreshold(float f);

            public native void read(@ByRef @Const opencv_core.FileNode fileNode);

            public native float thresholdStep();

            public native Params thresholdStep(float f);

            public native void write(@ByRef opencv_core.FileStorage fileStorage);

            static {
                Loader.load();
            }

            public Params(Pointer p) {
                super(p);
            }

            public Params(long size) {
                super((Pointer) null);
                allocateArray(size);
            }

            public Params position(long position) {
                return (Params) super.position(position);
            }

            public Params() {
                super((Pointer) null);
                allocate();
            }
        }
    }

    @Namespace("cv")
    public static class KAZE extends Feature2D {
        public static final int DIFF_CHARBONNIER = 3;
        public static final int DIFF_PM_G1 = 0;
        public static final int DIFF_PM_G2 = 1;
        public static final int DIFF_WEICKERT = 2;

        @opencv_core.Ptr
        public static native KAZE create();

        @opencv_core.Ptr
        public static native KAZE create(@Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2, float f, int i, int i2, @Cast({"cv::KAZE::DiffusivityType"}) int i3);

        @opencv_core.Str
        public native BytePointer getDefaultName();

        @Cast({"cv::KAZE::DiffusivityType"})
        public native int getDiffusivity();

        @Cast({"bool"})
        public native boolean getExtended();

        public native int getNOctaveLayers();

        public native int getNOctaves();

        public native double getThreshold();

        @Cast({"bool"})
        public native boolean getUpright();

        public native void setDiffusivity(@Cast({"cv::KAZE::DiffusivityType"}) int i);

        public native void setExtended(@Cast({"bool"}) boolean z);

        public native void setNOctaveLayers(int i);

        public native void setNOctaves(int i);

        public native void setThreshold(double d);

        public native void setUpright(@Cast({"bool"}) boolean z);

        static {
            Loader.load();
        }

        public KAZE(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class AKAZE extends Feature2D {
        public static final int DESCRIPTOR_KAZE = 3;
        public static final int DESCRIPTOR_KAZE_UPRIGHT = 2;
        public static final int DESCRIPTOR_MLDB = 5;
        public static final int DESCRIPTOR_MLDB_UPRIGHT = 4;

        @opencv_core.Ptr
        public static native AKAZE create();

        @opencv_core.Ptr
        public static native AKAZE create(@Cast({"cv::AKAZE::DescriptorType"}) int i, int i2, int i3, float f, int i4, int i5, @Cast({"cv::KAZE::DiffusivityType"}) int i6);

        @opencv_core.Str
        public native BytePointer getDefaultName();

        public native int getDescriptorChannels();

        public native int getDescriptorSize();

        @Cast({"cv::AKAZE::DescriptorType"})
        public native int getDescriptorType();

        @Cast({"cv::KAZE::DiffusivityType"})
        public native int getDiffusivity();

        public native int getNOctaveLayers();

        public native int getNOctaves();

        public native double getThreshold();

        public native void setDescriptorChannels(int i);

        public native void setDescriptorSize(int i);

        public native void setDescriptorType(@Cast({"cv::AKAZE::DescriptorType"}) int i);

        public native void setDiffusivity(@Cast({"cv::KAZE::DiffusivityType"}) int i);

        public native void setNOctaveLayers(int i);

        public native void setNOctaves(int i);

        public native void setThreshold(double d);

        static {
            Loader.load();
        }

        public AKAZE(Pointer p) {
            super(p);
        }
    }

    @Name({"cv::Accumulator<unsigned char>"})
    public static class Accumulator extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        static {
            Loader.load();
        }

        public Accumulator() {
            super((Pointer) null);
            allocate();
        }

        public Accumulator(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Accumulator(Pointer p) {
            super(p);
        }

        public Accumulator position(long position) {
            return (Accumulator) super.position(position);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class DescriptorMatcher extends opencv_core.Algorithm {
        public static final int BRUTEFORCE = 2;
        public static final int BRUTEFORCE_HAMMING = 4;
        public static final int BRUTEFORCE_HAMMINGLUT = 5;
        public static final int BRUTEFORCE_L1 = 3;
        public static final int BRUTEFORCE_SL2 = 6;
        public static final int FLANNBASED = 1;

        @opencv_core.Ptr
        public static native DescriptorMatcher create(@Cast({"const cv::DescriptorMatcher::MatcherType"}) int i);

        @opencv_core.Ptr
        public static native DescriptorMatcher create(@opencv_core.Str String str);

        @opencv_core.Ptr
        public static native DescriptorMatcher create(@opencv_core.Str BytePointer bytePointer);

        public native void add(@ByVal opencv_core.GpuMatVector gpuMatVector);

        public native void add(@ByVal opencv_core.MatVector matVector);

        public native void add(@ByVal opencv_core.UMatVector uMatVector);

        public native void clear();

        @opencv_core.Ptr
        public native DescriptorMatcher clone();

        @opencv_core.Ptr
        public native DescriptorMatcher clone(@Cast({"bool"}) boolean z);

        @Cast({"bool"})
        public native boolean empty();

        @ByRef
        @Const
        public native opencv_core.MatVector getTrainDescriptors();

        @Cast({"bool"})
        public native boolean isMaskSupported();

        public native void knnMatch(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, int i);

        public native void knnMatch(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, int i, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector, @Cast({"bool"}) boolean z);

        public native void knnMatch(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, int i, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector, @Cast({"bool"}) boolean z);

        public native void knnMatch(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, int i, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector, @Cast({"bool"}) boolean z);

        public native void knnMatch(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, int i);

        public native void knnMatch(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, int i, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat3, @Cast({"bool"}) boolean z);

        public native void knnMatch(@ByVal opencv_core.Mat mat, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, int i);

        public native void knnMatch(@ByVal opencv_core.Mat mat, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, int i, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector, @Cast({"bool"}) boolean z);

        public native void knnMatch(@ByVal opencv_core.Mat mat, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, int i, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector, @Cast({"bool"}) boolean z);

        public native void knnMatch(@ByVal opencv_core.Mat mat, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, int i, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector, @Cast({"bool"}) boolean z);

        public native void knnMatch(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, int i);

        public native void knnMatch(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, int i, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat3, @Cast({"bool"}) boolean z);

        public native void knnMatch(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, int i);

        public native void knnMatch(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, int i, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector, @Cast({"bool"}) boolean z);

        public native void knnMatch(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, int i, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector, @Cast({"bool"}) boolean z);

        public native void knnMatch(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, int i, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector, @Cast({"bool"}) boolean z);

        public native void knnMatch(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, int i);

        public native void knnMatch(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, int i, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat3, @Cast({"bool"}) boolean z);

        public native void match(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.DMatchVector dMatchVector);

        public native void match(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.DMatchVector dMatchVector, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector);

        public native void match(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.DMatchVector dMatchVector, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector);

        public native void match(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.DMatchVector dMatchVector, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector);

        public native void match(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef opencv_core.DMatchVector dMatchVector);

        public native void match(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef opencv_core.DMatchVector dMatchVector, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat3);

        public native void match(@ByVal opencv_core.Mat mat, @ByRef opencv_core.DMatchVector dMatchVector);

        public native void match(@ByVal opencv_core.Mat mat, @ByRef opencv_core.DMatchVector dMatchVector, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector);

        public native void match(@ByVal opencv_core.Mat mat, @ByRef opencv_core.DMatchVector dMatchVector, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector);

        public native void match(@ByVal opencv_core.Mat mat, @ByRef opencv_core.DMatchVector dMatchVector, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector);

        public native void match(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef opencv_core.DMatchVector dMatchVector);

        public native void match(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef opencv_core.DMatchVector dMatchVector, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat3);

        public native void match(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.DMatchVector dMatchVector);

        public native void match(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.DMatchVector dMatchVector, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector);

        public native void match(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.DMatchVector dMatchVector, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector);

        public native void match(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.DMatchVector dMatchVector, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector);

        public native void match(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef opencv_core.DMatchVector dMatchVector);

        public native void match(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef opencv_core.DMatchVector dMatchVector, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat3);

        public native void radiusMatch(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, float f);

        public native void radiusMatch(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, float f, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector, @Cast({"bool"}) boolean z);

        public native void radiusMatch(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, float f, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector, @Cast({"bool"}) boolean z);

        public native void radiusMatch(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, float f, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector, @Cast({"bool"}) boolean z);

        public native void radiusMatch(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, float f);

        public native void radiusMatch(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, float f, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.GpuMat gpuMat3, @Cast({"bool"}) boolean z);

        public native void radiusMatch(@ByVal opencv_core.Mat mat, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, float f);

        public native void radiusMatch(@ByVal opencv_core.Mat mat, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, float f, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector, @Cast({"bool"}) boolean z);

        public native void radiusMatch(@ByVal opencv_core.Mat mat, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, float f, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector, @Cast({"bool"}) boolean z);

        public native void radiusMatch(@ByVal opencv_core.Mat mat, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, float f, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector, @Cast({"bool"}) boolean z);

        public native void radiusMatch(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, float f);

        public native void radiusMatch(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, float f, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.Mat mat3, @Cast({"bool"}) boolean z);

        public native void radiusMatch(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, float f);

        public native void radiusMatch(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, float f, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.GpuMatVector gpuMatVector, @Cast({"bool"}) boolean z);

        public native void radiusMatch(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, float f, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.MatVector matVector, @Cast({"bool"}) boolean z);

        public native void radiusMatch(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, float f, @ByVal(nullValue = "cv::InputArrayOfArrays(cv::noArray())") opencv_core.UMatVector uMatVector, @Cast({"bool"}) boolean z);

        public native void radiusMatch(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, float f);

        public native void radiusMatch(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef opencv_core.DMatchVectorVector dMatchVectorVector, float f, @ByVal(nullValue = "cv::InputArray(cv::noArray())") opencv_core.UMat uMat3, @Cast({"bool"}) boolean z);

        public native void read(@opencv_core.Str String str);

        public native void read(@opencv_core.Str BytePointer bytePointer);

        public native void read(@ByRef @Const opencv_core.FileNode fileNode);

        public native void train();

        public native void write(@opencv_core.Str String str);

        public native void write(@opencv_core.Str BytePointer bytePointer);

        public native void write(@ByRef opencv_core.FileStorage fileStorage);

        public native void write(@opencv_core.Ptr opencv_core.FileStorage fileStorage, @opencv_core.Str String str);

        public native void write(@opencv_core.Ptr opencv_core.FileStorage fileStorage, @opencv_core.Str BytePointer bytePointer);

        static {
            Loader.load();
        }

        public DescriptorMatcher(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class BFMatcher extends DescriptorMatcher {
        private native void allocate();

        private native void allocate(int i, @Cast({"bool"}) boolean z);

        private native void allocateArray(long j);

        @opencv_core.Ptr
        public static native BFMatcher create();

        @opencv_core.Ptr
        public static native BFMatcher create(int i, @Cast({"bool"}) boolean z);

        @opencv_core.Ptr
        public native DescriptorMatcher clone();

        @opencv_core.Ptr
        public native DescriptorMatcher clone(@Cast({"bool"}) boolean z);

        @Cast({"bool"})
        public native boolean isMaskSupported();

        static {
            Loader.load();
        }

        public BFMatcher(Pointer p) {
            super(p);
        }

        public BFMatcher(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public BFMatcher position(long position) {
            return (BFMatcher) super.position(position);
        }

        public BFMatcher(int normType, @Cast({"bool"}) boolean crossCheck) {
            super((Pointer) null);
            allocate(normType, crossCheck);
        }

        public BFMatcher() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class FlannBasedMatcher extends DescriptorMatcher {
        private native void allocate();

        private native void allocate(@opencv_core.Ptr opencv_flann.IndexParams indexParams, @opencv_core.Ptr opencv_flann.SearchParams searchParams);

        private native void allocateArray(long j);

        @opencv_core.Ptr
        public static native FlannBasedMatcher create();

        public native void add(@ByVal opencv_core.GpuMatVector gpuMatVector);

        public native void add(@ByVal opencv_core.MatVector matVector);

        public native void add(@ByVal opencv_core.UMatVector uMatVector);

        public native void clear();

        @opencv_core.Ptr
        public native DescriptorMatcher clone();

        @opencv_core.Ptr
        public native DescriptorMatcher clone(@Cast({"bool"}) boolean z);

        @Cast({"bool"})
        public native boolean isMaskSupported();

        public native void read(@ByRef @Const opencv_core.FileNode fileNode);

        public native void train();

        public native void write(@ByRef opencv_core.FileStorage fileStorage);

        static {
            Loader.load();
        }

        public FlannBasedMatcher(Pointer p) {
            super(p);
        }

        public FlannBasedMatcher(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public FlannBasedMatcher position(long position) {
            return (FlannBasedMatcher) super.position(position);
        }

        public FlannBasedMatcher(@opencv_core.Ptr opencv_flann.IndexParams indexParams, @opencv_core.Ptr opencv_flann.SearchParams searchParams) {
            super((Pointer) null);
            allocate(indexParams, searchParams);
        }

        public FlannBasedMatcher() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class BOWTrainer extends Pointer {
        public native void add(@ByRef @Const opencv_core.Mat mat);

        public native void clear();

        @ByVal
        public native opencv_core.Mat cluster();

        @ByVal
        public native opencv_core.Mat cluster(@ByRef @Const opencv_core.Mat mat);

        public native int descriptorsCount();

        @ByRef
        @Const
        public native opencv_core.MatVector getDescriptors();

        static {
            Loader.load();
        }

        public BOWTrainer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class BOWKMeansTrainer extends BOWTrainer {
        private native void allocate(int i);

        private native void allocate(int i, @ByRef(nullValue = "cv::TermCriteria()") @Const opencv_core.TermCriteria termCriteria, int i2, int i3);

        @ByVal
        public native opencv_core.Mat cluster();

        @ByVal
        public native opencv_core.Mat cluster(@ByRef @Const opencv_core.Mat mat);

        static {
            Loader.load();
        }

        public BOWKMeansTrainer(Pointer p) {
            super(p);
        }

        public BOWKMeansTrainer(int clusterCount, @ByRef(nullValue = "cv::TermCriteria()") @Const opencv_core.TermCriteria termcrit, int attempts, int flags) {
            super((Pointer) null);
            allocate(clusterCount, termcrit, attempts, flags);
        }

        public BOWKMeansTrainer(int clusterCount) {
            super((Pointer) null);
            allocate(clusterCount);
        }
    }

    @Namespace("cv")
    @NoOffset
    public static class BOWImgDescriptorExtractor extends Pointer {
        private native void allocate(@opencv_core.Ptr DescriptorMatcher descriptorMatcher);

        private native void allocate(@opencv_core.Ptr @Cast({"cv::DescriptorExtractor*"}) Feature2D feature2D, @opencv_core.Ptr DescriptorMatcher descriptorMatcher);

        public native void compute(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void compute(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, opencv_core.IntVectorVector intVectorVector);

        public native void compute(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.GpuMat gpuMat2);

        public native void compute(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.GpuMat gpuMat2, opencv_core.IntVectorVector intVectorVector, opencv_core.Mat mat);

        public native void compute(@ByVal opencv_core.Mat mat, @ByRef opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.Mat mat2);

        public native void compute(@ByVal opencv_core.Mat mat, @ByRef opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.Mat mat2, opencv_core.IntVectorVector intVectorVector, opencv_core.Mat mat3);

        public native void compute(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void compute(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, opencv_core.IntVectorVector intVectorVector);

        public native void compute(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.UMat uMat2);

        public native void compute(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.KeyPointVector keyPointVector, @ByVal opencv_core.UMat uMat2, opencv_core.IntVectorVector intVectorVector, opencv_core.Mat mat);

        public native void compute(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void compute(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, opencv_core.IntVectorVector intVectorVector);

        public native int descriptorSize();

        public native int descriptorType();

        @ByRef
        @Const
        public native opencv_core.Mat getVocabulary();

        public native void setVocabulary(@ByRef @Const opencv_core.Mat mat);

        static {
            Loader.load();
        }

        public BOWImgDescriptorExtractor(Pointer p) {
            super(p);
        }

        public BOWImgDescriptorExtractor(@opencv_core.Ptr @Cast({"cv::DescriptorExtractor*"}) Feature2D dextractor, @opencv_core.Ptr DescriptorMatcher dmatcher) {
            super((Pointer) null);
            allocate(dextractor, dmatcher);
        }

        public BOWImgDescriptorExtractor(@opencv_core.Ptr DescriptorMatcher dmatcher) {
            super((Pointer) null);
            allocate(dmatcher);
        }
    }
}
