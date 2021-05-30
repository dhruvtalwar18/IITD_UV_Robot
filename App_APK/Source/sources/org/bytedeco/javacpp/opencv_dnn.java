package org.bytedeco.javacpp;

import java.nio.ByteBuffer;
import java.nio.DoubleBuffer;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.Arrays;
import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Const;
import org.bytedeco.javacpp.annotation.Convention;
import org.bytedeco.javacpp.annotation.Index;
import org.bytedeco.javacpp.annotation.Name;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.annotation.NoOffset;
import org.bytedeco.javacpp.annotation.StdString;
import org.bytedeco.javacpp.annotation.StdVector;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.presets.opencv_core;
import org.xbill.DNS.TTL;

public class opencv_dnn extends org.bytedeco.javacpp.presets.opencv_dnn {
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
    public static final int OPENCV_DNN_API_VERSION = 20181221;

    @Namespace("cv::dnn")
    public static native void NMSBoxes(@ByRef @Const opencv_core.Rect2dVector rect2dVector, @StdVector FloatBuffer floatBuffer, float f, float f2, @StdVector IntBuffer intBuffer);

    @Namespace("cv::dnn")
    public static native void NMSBoxes(@ByRef @Const opencv_core.Rect2dVector rect2dVector, @StdVector FloatBuffer floatBuffer, float f, float f2, @StdVector IntBuffer intBuffer, float f3, int i);

    @Namespace("cv::dnn")
    public static native void NMSBoxes(@ByRef @Const opencv_core.Rect2dVector rect2dVector, @StdVector FloatPointer floatPointer, float f, float f2, @StdVector IntPointer intPointer);

    @Namespace("cv::dnn")
    public static native void NMSBoxes(@ByRef @Const opencv_core.Rect2dVector rect2dVector, @StdVector FloatPointer floatPointer, float f, float f2, @StdVector IntPointer intPointer, float f3, int i);

    @Namespace("cv::dnn")
    public static native void NMSBoxes(@ByRef @Const opencv_core.Rect2dVector rect2dVector, @StdVector float[] fArr, float f, float f2, @StdVector int[] iArr);

    @Namespace("cv::dnn")
    public static native void NMSBoxes(@ByRef @Const opencv_core.Rect2dVector rect2dVector, @StdVector float[] fArr, float f, float f2, @StdVector int[] iArr, float f3, int i);

    @Namespace("cv::dnn")
    public static native void NMSBoxes(@ByRef @Const opencv_core.RectVector rectVector, @StdVector FloatBuffer floatBuffer, float f, float f2, @StdVector IntBuffer intBuffer);

    @Namespace("cv::dnn")
    public static native void NMSBoxes(@ByRef @Const opencv_core.RectVector rectVector, @StdVector FloatBuffer floatBuffer, float f, float f2, @StdVector IntBuffer intBuffer, float f3, int i);

    @Namespace("cv::dnn")
    public static native void NMSBoxes(@ByRef @Const opencv_core.RectVector rectVector, @StdVector FloatPointer floatPointer, float f, float f2, @StdVector IntPointer intPointer);

    @Namespace("cv::dnn")
    public static native void NMSBoxes(@ByRef @Const opencv_core.RectVector rectVector, @StdVector FloatPointer floatPointer, float f, float f2, @StdVector IntPointer intPointer, float f3, int i);

    @Namespace("cv::dnn")
    public static native void NMSBoxes(@ByRef @Const opencv_core.RectVector rectVector, @StdVector float[] fArr, float f, float f2, @StdVector int[] iArr);

    @Namespace("cv::dnn")
    public static native void NMSBoxes(@ByRef @Const opencv_core.RectVector rectVector, @StdVector float[] fArr, float f, float f2, @StdVector int[] iArr, float f3, int i);

    @Namespace("cv::dnn")
    @Name({"NMSBoxes"})
    public static native void NMSBoxesRotated(@StdVector opencv_core.RotatedRect rotatedRect, @StdVector FloatBuffer floatBuffer, float f, float f2, @StdVector IntBuffer intBuffer);

    @Namespace("cv::dnn")
    @Name({"NMSBoxes"})
    public static native void NMSBoxesRotated(@StdVector opencv_core.RotatedRect rotatedRect, @StdVector FloatBuffer floatBuffer, float f, float f2, @StdVector IntBuffer intBuffer, float f3, int i);

    @Namespace("cv::dnn")
    @Name({"NMSBoxes"})
    public static native void NMSBoxesRotated(@StdVector opencv_core.RotatedRect rotatedRect, @StdVector FloatPointer floatPointer, float f, float f2, @StdVector IntPointer intPointer);

    @Namespace("cv::dnn")
    @Name({"NMSBoxes"})
    public static native void NMSBoxesRotated(@StdVector opencv_core.RotatedRect rotatedRect, @StdVector FloatPointer floatPointer, float f, float f2, @StdVector IntPointer intPointer, float f3, int i);

    @Namespace("cv::dnn")
    @Name({"NMSBoxes"})
    public static native void NMSBoxesRotated(@StdVector opencv_core.RotatedRect rotatedRect, @StdVector float[] fArr, float f, float f2, @StdVector int[] iArr);

    @Namespace("cv::dnn")
    @Name({"NMSBoxes"})
    public static native void NMSBoxesRotated(@StdVector opencv_core.RotatedRect rotatedRect, @StdVector float[] fArr, float f, float f2, @StdVector int[] iArr, float f3, int i);

    @Namespace("cv::dnn")
    @ByVal
    public static native opencv_core.Mat blobFromImage(@ByVal opencv_core.GpuMat gpuMat);

    @Namespace("cv::dnn")
    @ByVal
    public static native opencv_core.Mat blobFromImage(@ByVal opencv_core.GpuMat gpuMat, double d, @ByRef(nullValue = "cv::Size()") @Const opencv_core.Size size, @ByRef(nullValue = "cv::Scalar()") @Const opencv_core.Scalar scalar, @Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2, int i);

    @Namespace("cv::dnn")
    @ByVal
    public static native opencv_core.Mat blobFromImage(@ByVal opencv_core.Mat mat);

    @Namespace("cv::dnn")
    @ByVal
    public static native opencv_core.Mat blobFromImage(@ByVal opencv_core.Mat mat, double d, @ByRef(nullValue = "cv::Size()") @Const opencv_core.Size size, @ByRef(nullValue = "cv::Scalar()") @Const opencv_core.Scalar scalar, @Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2, int i);

    @Namespace("cv::dnn")
    @ByVal
    public static native opencv_core.Mat blobFromImage(@ByVal opencv_core.UMat uMat);

    @Namespace("cv::dnn")
    @ByVal
    public static native opencv_core.Mat blobFromImage(@ByVal opencv_core.UMat uMat, double d, @ByRef(nullValue = "cv::Size()") @Const opencv_core.Size size, @ByRef(nullValue = "cv::Scalar()") @Const opencv_core.Scalar scalar, @Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2, int i);

    @Namespace("cv::dnn")
    public static native void blobFromImage(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv::dnn")
    public static native void blobFromImage(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, double d, @ByRef(nullValue = "cv::Size()") @Const opencv_core.Size size, @ByRef(nullValue = "cv::Scalar()") @Const opencv_core.Scalar scalar, @Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2, int i);

    @Namespace("cv::dnn")
    public static native void blobFromImage(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv::dnn")
    public static native void blobFromImage(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, double d, @ByRef(nullValue = "cv::Size()") @Const opencv_core.Size size, @ByRef(nullValue = "cv::Scalar()") @Const opencv_core.Scalar scalar, @Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2, int i);

    @Namespace("cv::dnn")
    public static native void blobFromImage(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv::dnn")
    public static native void blobFromImage(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, double d, @ByRef(nullValue = "cv::Size()") @Const opencv_core.Size size, @ByRef(nullValue = "cv::Scalar()") @Const opencv_core.Scalar scalar, @Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2, int i);

    @Namespace("cv::dnn")
    @ByVal
    public static native opencv_core.Mat blobFromImages(@ByVal opencv_core.GpuMatVector gpuMatVector);

    @Namespace("cv::dnn")
    @ByVal
    public static native opencv_core.Mat blobFromImages(@ByVal opencv_core.GpuMatVector gpuMatVector, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByRef(nullValue = "cv::Scalar()") @Const opencv_core.Scalar scalar, @Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2, int i);

    @Namespace("cv::dnn")
    @ByVal
    public static native opencv_core.Mat blobFromImages(@ByVal opencv_core.MatVector matVector);

    @Namespace("cv::dnn")
    @ByVal
    public static native opencv_core.Mat blobFromImages(@ByVal opencv_core.MatVector matVector, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByRef(nullValue = "cv::Scalar()") @Const opencv_core.Scalar scalar, @Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2, int i);

    @Namespace("cv::dnn")
    @ByVal
    public static native opencv_core.Mat blobFromImages(@ByVal opencv_core.UMatVector uMatVector);

    @Namespace("cv::dnn")
    @ByVal
    public static native opencv_core.Mat blobFromImages(@ByVal opencv_core.UMatVector uMatVector, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByRef(nullValue = "cv::Scalar()") @Const opencv_core.Scalar scalar, @Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2, int i);

    @Namespace("cv::dnn")
    public static native void blobFromImages(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat);

    @Namespace("cv::dnn")
    public static native void blobFromImages(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMat gpuMat, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByRef(nullValue = "cv::Scalar()") @Const opencv_core.Scalar scalar, @Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2, int i);

    @Namespace("cv::dnn")
    public static native void blobFromImages(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat);

    @Namespace("cv::dnn")
    public static native void blobFromImages(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.Mat mat, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByRef(nullValue = "cv::Scalar()") @Const opencv_core.Scalar scalar, @Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2, int i);

    @Namespace("cv::dnn")
    public static native void blobFromImages(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat);

    @Namespace("cv::dnn")
    public static native void blobFromImages(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.UMat uMat, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByRef(nullValue = "cv::Scalar()") @Const opencv_core.Scalar scalar, @Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2, int i);

    @Namespace("cv::dnn")
    public static native void blobFromImages(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat);

    @Namespace("cv::dnn")
    public static native void blobFromImages(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.GpuMat gpuMat, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByRef(nullValue = "cv::Scalar()") @Const opencv_core.Scalar scalar, @Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2, int i);

    @Namespace("cv::dnn")
    public static native void blobFromImages(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat);

    @Namespace("cv::dnn")
    public static native void blobFromImages(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.Mat mat, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByRef(nullValue = "cv::Scalar()") @Const opencv_core.Scalar scalar, @Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2, int i);

    @Namespace("cv::dnn")
    public static native void blobFromImages(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat);

    @Namespace("cv::dnn")
    public static native void blobFromImages(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.UMat uMat, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByRef(nullValue = "cv::Scalar()") @Const opencv_core.Scalar scalar, @Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2, int i);

    @Namespace("cv::dnn")
    public static native void blobFromImages(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat);

    @Namespace("cv::dnn")
    public static native void blobFromImages(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.GpuMat gpuMat, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByRef(nullValue = "cv::Scalar()") @Const opencv_core.Scalar scalar, @Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2, int i);

    @Namespace("cv::dnn")
    public static native void blobFromImages(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat);

    @Namespace("cv::dnn")
    public static native void blobFromImages(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.Mat mat, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByRef(nullValue = "cv::Scalar()") @Const opencv_core.Scalar scalar, @Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2, int i);

    @Namespace("cv::dnn")
    public static native void blobFromImages(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat);

    @Namespace("cv::dnn")
    public static native void blobFromImages(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMat uMat, double d, @ByVal(nullValue = "cv::Size()") opencv_core.Size size, @ByRef(nullValue = "cv::Scalar()") @Const opencv_core.Scalar scalar, @Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2, int i);

    @Namespace("cv::dnn")
    public static native int clamp(int i, int i2);

    @Namespace("cv::dnn")
    public static native int clamp(int i, @ByRef @Const @StdVector IntPointer intPointer);

    @Namespace("cv::dnn")
    @ByVal
    public static native opencv_core.Range clamp(@ByRef @Const opencv_core.Range range, int i);

    @Namespace("cv::dnn")
    @StdVector
    @ByVal
    public static native IntPointer concat(@ByRef @Const @StdVector IntPointer intPointer, @ByRef @Const @StdVector IntPointer intPointer2);

    @Namespace("cv::dnn")
    @Cast({"std::vector<std::pair<cv::dnn::Backend,cv::dnn::Target> >*"})
    @ByVal
    public static native opencv_core.IntIntPairVector getAvailableBackends();

    @Namespace("cv::dnn")
    @Cast({"cv::dnn::Target*"})
    @StdVector
    public static native IntPointer getAvailableTargets(@Cast({"cv::dnn::Backend"}) int i);

    @Namespace("cv::dnn")
    @ByVal
    public static native opencv_core.Mat getPlane(@ByRef @Const opencv_core.Mat mat, int i, int i2);

    @Namespace("cv::dnn")
    public static native void imagesFromBlob(@ByRef @Const opencv_core.Mat mat, @ByVal opencv_core.GpuMatVector gpuMatVector);

    @Namespace("cv::dnn")
    public static native void imagesFromBlob(@ByRef @Const opencv_core.Mat mat, @ByVal opencv_core.MatVector matVector);

    @Namespace("cv::dnn")
    public static native void imagesFromBlob(@ByRef @Const opencv_core.Mat mat, @ByVal opencv_core.UMatVector uMatVector);

    @Namespace("cv::dnn")
    @Cast({"bool"})
    public static native boolean is_neg(int i);

    @Namespace("cv::dnn")
    public static native void print(@ByRef @Const @StdVector IntPointer intPointer);

    @Namespace("cv::dnn")
    public static native void print(@ByRef @Const @StdVector IntPointer intPointer, @opencv_core.Str String str);

    @Namespace("cv::dnn")
    public static native void print(@ByRef @Const @StdVector IntPointer intPointer, @opencv_core.Str BytePointer bytePointer);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNet(@opencv_core.Str String str);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNet(@opencv_core.Str String str, @opencv_core.Str String str2, @opencv_core.Str String str3);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNet(@opencv_core.Str String str, @Cast({"uchar*"}) @StdVector ByteBuffer byteBuffer);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNet(@opencv_core.Str String str, @Cast({"uchar*"}) @StdVector ByteBuffer byteBuffer, @Cast({"uchar*"}) @StdVector ByteBuffer byteBuffer2);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNet(@opencv_core.Str String str, @Cast({"uchar*"}) @StdVector BytePointer bytePointer);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNet(@opencv_core.Str String str, @Cast({"uchar*"}) @StdVector BytePointer bytePointer, @Cast({"uchar*"}) @StdVector BytePointer bytePointer2);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNet(@opencv_core.Str String str, @Cast({"uchar*"}) @StdVector byte[] bArr);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNet(@opencv_core.Str String str, @Cast({"uchar*"}) @StdVector byte[] bArr, @Cast({"uchar*"}) @StdVector byte[] bArr2);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNet(@opencv_core.Str BytePointer bytePointer);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNet(@opencv_core.Str BytePointer bytePointer, @Cast({"uchar*"}) @StdVector ByteBuffer byteBuffer);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNet(@opencv_core.Str BytePointer bytePointer, @Cast({"uchar*"}) @StdVector ByteBuffer byteBuffer, @Cast({"uchar*"}) @StdVector ByteBuffer byteBuffer2);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNet(@opencv_core.Str BytePointer bytePointer, @Cast({"uchar*"}) @StdVector BytePointer bytePointer2);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNet(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @opencv_core.Str BytePointer bytePointer3);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNet(@opencv_core.Str BytePointer bytePointer, @Cast({"uchar*"}) @StdVector byte[] bArr);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNet(@opencv_core.Str BytePointer bytePointer, @Cast({"uchar*"}) @StdVector byte[] bArr, @Cast({"uchar*"}) @StdVector byte[] bArr2);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromCaffe(@opencv_core.Str String str);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromCaffe(String str, @Cast({"size_t"}) long j);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromCaffe(String str, @Cast({"size_t"}) long j, String str2, @Cast({"size_t"}) long j2);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromCaffe(@opencv_core.Str String str, @opencv_core.Str String str2);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromCaffe(@Cast({"uchar*"}) @StdVector ByteBuffer byteBuffer);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromCaffe(@Cast({"uchar*"}) @StdVector ByteBuffer byteBuffer, @Cast({"uchar*"}) @StdVector ByteBuffer byteBuffer2);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromCaffe(@opencv_core.Str BytePointer bytePointer);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromCaffe(@Cast({"const char*"}) BytePointer bytePointer, @Cast({"size_t"}) long j);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromCaffe(@Cast({"const char*"}) BytePointer bytePointer, @Cast({"size_t"}) long j, @Cast({"const char*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j2);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromCaffe(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromCaffe(@Cast({"uchar*"}) @StdVector byte[] bArr);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromCaffe(@Cast({"uchar*"}) @StdVector byte[] bArr, @Cast({"uchar*"}) @StdVector byte[] bArr2);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromDarknet(@opencv_core.Str String str);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromDarknet(String str, @Cast({"size_t"}) long j);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromDarknet(String str, @Cast({"size_t"}) long j, String str2, @Cast({"size_t"}) long j2);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromDarknet(@opencv_core.Str String str, @opencv_core.Str String str2);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromDarknet(@Cast({"uchar*"}) @StdVector ByteBuffer byteBuffer);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromDarknet(@Cast({"uchar*"}) @StdVector ByteBuffer byteBuffer, @Cast({"uchar*"}) @StdVector ByteBuffer byteBuffer2);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromDarknet(@opencv_core.Str BytePointer bytePointer);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromDarknet(@Cast({"const char*"}) BytePointer bytePointer, @Cast({"size_t"}) long j);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromDarknet(@Cast({"const char*"}) BytePointer bytePointer, @Cast({"size_t"}) long j, @Cast({"const char*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j2);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromDarknet(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromDarknet(@Cast({"uchar*"}) @StdVector byte[] bArr);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromDarknet(@Cast({"uchar*"}) @StdVector byte[] bArr, @Cast({"uchar*"}) @StdVector byte[] bArr2);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromModelOptimizer(@opencv_core.Str String str, @opencv_core.Str String str2);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromModelOptimizer(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromONNX(@opencv_core.Str String str);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromONNX(@opencv_core.Str BytePointer bytePointer);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromTensorflow(@opencv_core.Str String str);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromTensorflow(String str, @Cast({"size_t"}) long j);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromTensorflow(String str, @Cast({"size_t"}) long j, String str2, @Cast({"size_t"}) long j2);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromTensorflow(@opencv_core.Str String str, @opencv_core.Str String str2);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromTensorflow(@Cast({"uchar*"}) @StdVector ByteBuffer byteBuffer);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromTensorflow(@Cast({"uchar*"}) @StdVector ByteBuffer byteBuffer, @Cast({"uchar*"}) @StdVector ByteBuffer byteBuffer2);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromTensorflow(@opencv_core.Str BytePointer bytePointer);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromTensorflow(@Cast({"const char*"}) BytePointer bytePointer, @Cast({"size_t"}) long j);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromTensorflow(@Cast({"const char*"}) BytePointer bytePointer, @Cast({"size_t"}) long j, @Cast({"const char*"}) BytePointer bytePointer2, @Cast({"size_t"}) long j2);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromTensorflow(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromTensorflow(@Cast({"uchar*"}) @StdVector byte[] bArr);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromTensorflow(@Cast({"uchar*"}) @StdVector byte[] bArr, @Cast({"uchar*"}) @StdVector byte[] bArr2);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromTorch(@opencv_core.Str String str);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromTorch(@opencv_core.Str String str, @Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromTorch(@opencv_core.Str BytePointer bytePointer);

    @Namespace("cv::dnn")
    @ByVal
    public static native Net readNetFromTorch(@opencv_core.Str BytePointer bytePointer, @Cast({"bool"}) boolean z, @Cast({"bool"}) boolean z2);

    @Namespace("cv::dnn")
    @ByVal
    public static native opencv_core.Mat readTensorFromONNX(@opencv_core.Str String str);

    @Namespace("cv::dnn")
    @ByVal
    public static native opencv_core.Mat readTensorFromONNX(@opencv_core.Str BytePointer bytePointer);

    @Namespace("cv::dnn")
    @ByVal
    public static native opencv_core.Mat readTorchBlob(@opencv_core.Str String str);

    @Namespace("cv::dnn")
    @ByVal
    public static native opencv_core.Mat readTorchBlob(@opencv_core.Str String str, @Cast({"bool"}) boolean z);

    @Namespace("cv::dnn")
    @ByVal
    public static native opencv_core.Mat readTorchBlob(@opencv_core.Str BytePointer bytePointer);

    @Namespace("cv::dnn")
    @ByVal
    public static native opencv_core.Mat readTorchBlob(@opencv_core.Str BytePointer bytePointer, @Cast({"bool"}) boolean z);

    @Namespace("cv::dnn")
    public static native void resetMyriadDevice();

    @Namespace("cv::dnn")
    @StdVector
    @ByVal
    public static native IntPointer shape(int i);

    @Namespace("cv::dnn")
    @StdVector
    @ByVal
    public static native IntPointer shape(int i, int i2, int i3, int i4);

    @Namespace("cv::dnn")
    @StdVector
    @ByVal
    public static native IntPointer shape(@Const IntBuffer intBuffer, int i);

    @Namespace("cv::dnn")
    @StdVector
    @ByVal
    public static native IntPointer shape(@Const IntPointer intPointer, int i);

    @Namespace("cv::dnn")
    @StdVector
    @ByVal
    public static native IntPointer shape(@ByRef @Const opencv_core.Mat mat);

    @Namespace("cv::dnn")
    @StdVector
    @ByVal
    public static native IntPointer shape(@ByRef @Const opencv_core.MatSize matSize);

    @Namespace("cv::dnn")
    @StdVector
    @ByVal
    public static native IntPointer shape(@ByRef @Const opencv_core.UMat uMat);

    @Namespace("cv::dnn")
    @StdVector
    @ByVal
    public static native IntPointer shape(@Const int[] iArr, int i);

    @ByRef
    @Cast({"std::ostream*"})
    @Name({"operator <<"})
    @Namespace("cv::dnn")
    public static native Pointer shiftLeft(@ByRef @Cast({"std::ostream*"}) Pointer pointer, @ByRef @Const @StdVector IntPointer intPointer);

    @Namespace("cv::dnn")
    public static native void shrinkCaffeModel(@opencv_core.Str String str, @opencv_core.Str String str2);

    @Namespace("cv::dnn")
    public static native void shrinkCaffeModel(@opencv_core.Str String str, @opencv_core.Str String str2, @ByRef(nullValue = "std::vector<cv::String>()") @Const opencv_core.StringVector stringVector);

    @Namespace("cv::dnn")
    public static native void shrinkCaffeModel(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

    @Namespace("cv::dnn")
    public static native void shrinkCaffeModel(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @ByRef(nullValue = "std::vector<cv::String>()") @Const opencv_core.StringVector stringVector);

    @Namespace("cv::dnn")
    @ByVal
    public static native opencv_core.Mat slice(@ByRef @Const opencv_core.Mat mat, @ByRef @Const _Range _range);

    @Namespace("cv::dnn")
    @ByVal
    public static native opencv_core.Mat slice(@ByRef @Const opencv_core.Mat mat, @ByRef @Const _Range _range, @ByRef @Const _Range _range2);

    @Namespace("cv::dnn")
    @ByVal
    public static native opencv_core.Mat slice(@ByRef @Const opencv_core.Mat mat, @ByRef @Const _Range _range, @ByRef @Const _Range _range2, @ByRef @Const _Range _range3);

    @Namespace("cv::dnn")
    @ByVal
    public static native opencv_core.Mat slice(@ByRef @Const opencv_core.Mat mat, @ByRef @Const _Range _range, @ByRef @Const _Range _range2, @ByRef @Const _Range _range3, @ByRef @Const _Range _range4);

    @Namespace("cv::dnn")
    @StdString
    public static native String toString(@ByRef @Const @StdVector IntPointer intPointer, @opencv_core.Str String str);

    @Namespace("cv::dnn")
    @StdString
    public static native BytePointer toString(@ByRef @Const @StdVector IntPointer intPointer);

    @Namespace("cv::dnn")
    @StdString
    public static native BytePointer toString(@ByRef @Const @StdVector IntPointer intPointer, @opencv_core.Str BytePointer bytePointer);

    @Namespace("cv::dnn")
    public static native int total(@ByRef @Const @StdVector IntPointer intPointer);

    @Namespace("cv::dnn")
    public static native int total(@ByRef @Const @StdVector IntPointer intPointer, int i, int i2);

    @Namespace("cv::dnn")
    public static native void writeTextGraph(@opencv_core.Str String str, @opencv_core.Str String str2);

    @Namespace("cv::dnn")
    public static native void writeTextGraph(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

    static {
        Loader.load();
    }

    @Name({"std::vector<cv::dnn::MatShape>"})
    public static class MatShapeVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @Index(function = "at")
        @StdVector
        public native IntPointer get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @StdVector IntPointer intPointer);

        public native MatShapeVector put(@Cast({"size_t"}) long j, IntPointer intPointer);

        @ByRef
        @Name({"operator="})
        public native MatShapeVector put(@ByRef MatShapeVector matShapeVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public MatShapeVector(Pointer p) {
            super(p);
        }

        public MatShapeVector(IntPointer value) {
            this(1);
            put(0, value);
        }

        public MatShapeVector(IntPointer... array) {
            this((long) array.length);
            put(array);
        }

        public MatShapeVector() {
            allocate();
        }

        public MatShapeVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @Const
            @StdVector
            @Name({"operator*"})
            public native IntPointer get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public IntPointer[] get() {
            IntPointer[] array = new IntPointer[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return Arrays.toString(get());
        }

        public IntPointer pop_back() {
            long size = size();
            IntPointer value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public MatShapeVector push_back(IntPointer value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public MatShapeVector put(IntPointer value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public MatShapeVector put(IntPointer... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<std::vector<cv::dnn::MatShape> >"})
    public static class MatShapeVectorVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @ByRef
        @Index(function = "at")
        public native MatShapeVector get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @ByRef MatShapeVector matShapeVector);

        public native MatShapeVectorVector put(@Cast({"size_t"}) long j, MatShapeVector matShapeVector);

        @ByRef
        @Name({"operator="})
        public native MatShapeVectorVector put(@ByRef MatShapeVectorVector matShapeVectorVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public MatShapeVectorVector(Pointer p) {
            super(p);
        }

        public MatShapeVectorVector(MatShapeVector value) {
            this(1);
            put(0, value);
        }

        public MatShapeVectorVector(MatShapeVector... array) {
            this((long) array.length);
            put(array);
        }

        public MatShapeVectorVector() {
            allocate();
        }

        public MatShapeVectorVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @ByRef
            @Const
            @Name({"operator*"})
            public native MatShapeVector get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public MatShapeVector[] get() {
            MatShapeVector[] array = new MatShapeVector[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return Arrays.toString(get());
        }

        public MatShapeVector pop_back() {
            long size = size();
            MatShapeVector value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public MatShapeVectorVector push_back(MatShapeVector value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public MatShapeVectorVector put(MatShapeVector value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public MatShapeVectorVector put(MatShapeVector... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<std::vector<cv::Range> >"})
    public static class RangeVectorVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByRef
        @Index(function = "at")
        public native opencv_core.Range get(@Cast({"size_t"}) long j, @Cast({"size_t"}) long j2);

        public native RangeVectorVector put(@Cast({"size_t"}) long j, @Cast({"size_t"}) long j2, opencv_core.Range range);

        @ByRef
        @Name({"operator="})
        public native RangeVectorVector put(@ByRef RangeVectorVector rangeVectorVector);

        public native void resize(@Cast({"size_t"}) long j);

        @Index(function = "at")
        public native void resize(@Cast({"size_t"}) long j, @Cast({"size_t"}) long j2);

        public native long size();

        @Index(function = "at")
        public native long size(@Cast({"size_t"}) long j);

        static {
            Loader.load();
        }

        public RangeVectorVector(Pointer p) {
            super(p);
        }

        public RangeVectorVector(opencv_core.Range[]... array) {
            this((long) array.length);
            put(array);
        }

        public RangeVectorVector() {
            allocate();
        }

        public RangeVectorVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        public boolean empty(@Cast({"size_t"}) long i) {
            return size(i) == 0;
        }

        public void clear(@Cast({"size_t"}) long i) {
            resize(i, 0);
        }

        public opencv_core.Range[][] get() {
            opencv_core.Range[][] array = new opencv_core.Range[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)][];
            for (int i = 0; i < array.length; i++) {
                array[i] = new opencv_core.Range[(size((long) i) < TTL.MAX_VALUE ? (int) size((long) i) : Integer.MAX_VALUE)];
                for (int j = 0; j < array[i].length; j++) {
                    array[i][j] = get((long) i, (long) j);
                }
            }
            return array;
        }

        public String toString() {
            return Arrays.deepToString(get());
        }

        public RangeVectorVector put(opencv_core.Range[]... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                if (size((long) i) != ((long) array[i].length)) {
                    resize((long) i, (long) array[i].length);
                }
                for (int j = 0; j < array[i].length; j++) {
                    put((long) i, (long) j, array[i][j]);
                }
            }
            return this;
        }
    }

    @Name({"std::vector<cv::Mat*>"})
    public static class MatPointerVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @Index(function = "at")
        public native opencv_core.Mat get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, opencv_core.Mat mat);

        public native MatPointerVector put(@Cast({"size_t"}) long j, opencv_core.Mat mat);

        @ByRef
        @Name({"operator="})
        public native MatPointerVector put(@ByRef MatPointerVector matPointerVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public MatPointerVector(Pointer p) {
            super(p);
        }

        public MatPointerVector(opencv_core.Mat value) {
            this(1);
            put(0, value);
        }

        public MatPointerVector(opencv_core.Mat... array) {
            this((long) array.length);
            put(array);
        }

        public MatPointerVector() {
            allocate();
        }

        public MatPointerVector(long n) {
            allocate(n);
        }

        public boolean empty() {
            return size() == 0;
        }

        public void clear() {
            resize(0);
        }

        @Name({"iterator"})
        @NoOffset
        public static class Iterator extends Pointer {
            @Name({"operator=="})
            public native boolean equals(@ByRef Iterator iterator);

            @Const
            @Name({"operator*"})
            public native opencv_core.Mat get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public opencv_core.Mat[] get() {
            opencv_core.Mat[] array = new opencv_core.Mat[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return Arrays.toString(get());
        }

        public opencv_core.Mat pop_back() {
            long size = size();
            opencv_core.Mat value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public MatPointerVector push_back(opencv_core.Mat value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public MatPointerVector put(opencv_core.Mat value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public MatPointerVector put(opencv_core.Mat... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Namespace("cv::dnn")
    @NoOffset
    public static class DictValue extends Pointer {
        private native void allocate();

        private native void allocate(double d);

        private native void allocate(int i);

        private native void allocate(@Cast({"int64"}) long j);

        private native void allocate(@opencv_core.Str String str);

        private native void allocate(@opencv_core.Str BytePointer bytePointer);

        private native void allocate(@ByRef @Const DictValue dictValue);

        private native void allocate(@Cast({"bool"}) boolean z);

        public native int getIntValue();

        public native int getIntValue(int i);

        public native double getRealValue();

        public native double getRealValue(int i);

        @opencv_core.Str
        public native BytePointer getStringValue();

        @opencv_core.Str
        public native BytePointer getStringValue(int i);

        @Cast({"bool"})
        public native boolean isInt();

        @Cast({"bool"})
        public native boolean isReal();

        @Cast({"bool"})
        public native boolean isString();

        @ByRef
        @Name({"operator ="})
        public native DictValue put(@ByRef @Const DictValue dictValue);

        public native int size();

        static {
            Loader.load();
        }

        public DictValue(Pointer p) {
            super(p);
        }

        public DictValue(@ByRef @Const DictValue r) {
            super((Pointer) null);
            allocate(r);
        }

        public DictValue(@Cast({"bool"}) boolean i) {
            super((Pointer) null);
            allocate(i);
        }

        public DictValue(@Cast({"int64"}) long i) {
            super((Pointer) null);
            allocate(i);
        }

        public DictValue() {
            super((Pointer) null);
            allocate();
        }

        public DictValue(int i) {
            super((Pointer) null);
            allocate(i);
        }

        public DictValue(double p) {
            super((Pointer) null);
            allocate(p);
        }

        public DictValue(@opencv_core.Str BytePointer s) {
            super((Pointer) null);
            allocate(s);
        }

        public DictValue(@opencv_core.Str String s) {
            super((Pointer) null);
            allocate(s);
        }
    }

    @Namespace("cv::dnn")
    public static class Dict extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        public native void erase(@opencv_core.Str String str);

        public native void erase(@opencv_core.Str BytePointer bytePointer);

        @ByRef
        @Const
        public native DictValue get(@opencv_core.Str String str);

        @ByRef
        @Const
        public native DictValue get(@opencv_core.Str BytePointer bytePointer);

        @Cast({"bool"})
        public native boolean has(@opencv_core.Str String str);

        @Cast({"bool"})
        public native boolean has(@opencv_core.Str BytePointer bytePointer);

        public native DictValue ptr(@opencv_core.Str String str);

        public native DictValue ptr(@opencv_core.Str BytePointer bytePointer);

        static {
            Loader.load();
        }

        public Dict() {
            super((Pointer) null);
            allocate();
        }

        public Dict(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Dict(Pointer p) {
            super(p);
        }

        public Dict position(long position) {
            return (Dict) super.position(position);
        }
    }

    @Namespace("cv::dnn")
    public static class BlankLayer extends Layer {
        @opencv_core.Ptr
        public static native Layer create(@ByRef @Const LayerParams layerParams);

        static {
            Loader.load();
        }

        public BlankLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    public static class ConstLayer extends Layer {
        private native void allocate();

        private native void allocateArray(long j);

        @opencv_core.Ptr
        public static native Layer create(@ByRef @Const LayerParams layerParams);

        static {
            Loader.load();
        }

        public ConstLayer() {
            super((Pointer) null);
            allocate();
        }

        public ConstLayer(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public ConstLayer(Pointer p) {
            super(p);
        }

        public ConstLayer position(long position) {
            return (ConstLayer) super.position(position);
        }
    }

    @Namespace("cv::dnn")
    public static class LSTMLayer extends Layer {
        @opencv_core.Ptr
        public static native LSTMLayer create(@ByRef @Const LayerParams layerParams);

        public native int inputNameToIndex(@opencv_core.Str String str);

        public native int inputNameToIndex(@opencv_core.Str BytePointer bytePointer);

        public native int outputNameToIndex(@opencv_core.Str String str);

        public native int outputNameToIndex(@opencv_core.Str BytePointer bytePointer);

        public native void setOutShape();

        public native void setOutShape(@ByRef(nullValue = "cv::dnn::MatShape()") @Const @StdVector IntPointer intPointer);

        @Deprecated
        public native void setProduceCellOutput();

        @Deprecated
        public native void setProduceCellOutput(@Cast({"bool"}) boolean z);

        @Deprecated
        public native void setUseTimstampsDim();

        @Deprecated
        public native void setUseTimstampsDim(@Cast({"bool"}) boolean z);

        @Deprecated
        public native void setWeights(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2, @ByRef @Const opencv_core.Mat mat3);

        static {
            Loader.load();
        }

        public LSTMLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    public static class RNNLayer extends Layer {
        @opencv_core.Ptr
        public static native RNNLayer create(@ByRef @Const LayerParams layerParams);

        public native void setProduceHiddenOutput();

        public native void setProduceHiddenOutput(@Cast({"bool"}) boolean z);

        public native void setWeights(@ByRef @Const opencv_core.Mat mat, @ByRef @Const opencv_core.Mat mat2, @ByRef @Const opencv_core.Mat mat3, @ByRef @Const opencv_core.Mat mat4, @ByRef @Const opencv_core.Mat mat5);

        static {
            Loader.load();
        }

        public RNNLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    @NoOffset
    public static class BaseConvolutionLayer extends Layer {
        @ByRef
        public native opencv_core.Size adjustPad();

        public native BaseConvolutionLayer adjustPad(opencv_core.Size size);

        @ByRef
        public native opencv_core.Size dilation();

        public native BaseConvolutionLayer dilation(opencv_core.Size size);

        @ByRef
        public native opencv_core.Size kernel();

        public native BaseConvolutionLayer kernel(opencv_core.Size size);

        public native int numOutput();

        public native BaseConvolutionLayer numOutput(int i);

        @ByRef
        public native opencv_core.Size pad();

        public native BaseConvolutionLayer pad(opencv_core.Size size);

        @opencv_core.Str
        public native BytePointer padMode();

        public native BaseConvolutionLayer padMode(BytePointer bytePointer);

        @ByRef
        public native opencv_core.Size stride();

        public native BaseConvolutionLayer stride(opencv_core.Size size);

        static {
            Loader.load();
        }

        public BaseConvolutionLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    public static class ConvolutionLayer extends BaseConvolutionLayer {
        @opencv_core.Ptr
        public static native BaseConvolutionLayer create(@ByRef @Const LayerParams layerParams);

        static {
            Loader.load();
        }

        public ConvolutionLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    public static class DeconvolutionLayer extends BaseConvolutionLayer {
        @opencv_core.Ptr
        public static native BaseConvolutionLayer create(@ByRef @Const LayerParams layerParams);

        static {
            Loader.load();
        }

        public DeconvolutionLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    @NoOffset
    public static class LRNLayer extends Layer {
        @opencv_core.Ptr
        public static native LRNLayer create(@ByRef @Const LayerParams layerParams);

        public native float alpha();

        public native LRNLayer alpha(float f);

        public native float beta();

        public native LRNLayer beta(float f);

        public native float bias();

        public native LRNLayer bias(float f);

        @Name({"type"})
        public native int lrnType();

        public native LRNLayer lrnType(int i);

        public native LRNLayer normBySize(boolean z);

        @Cast({"bool"})
        public native boolean normBySize();

        public native int size();

        public native LRNLayer size(int i);

        static {
            Loader.load();
        }

        public LRNLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    @NoOffset
    public static class PoolingLayer extends Layer {
        @opencv_core.Ptr
        public static native PoolingLayer create(@ByRef @Const LayerParams layerParams);

        public native PoolingLayer avePoolPaddedArea(boolean z);

        @Cast({"bool"})
        public native boolean avePoolPaddedArea();

        public native PoolingLayer ceilMode(boolean z);

        @Cast({"bool"})
        public native boolean ceilMode();

        public native PoolingLayer computeMaxIdx(boolean z);

        @Cast({"bool"})
        public native boolean computeMaxIdx();

        public native PoolingLayer globalPooling(boolean z);

        @Cast({"bool"})
        public native boolean globalPooling();

        @ByRef
        public native opencv_core.Size kernel();

        public native PoolingLayer kernel(opencv_core.Size size);

        @ByRef
        @Deprecated
        public native opencv_core.Size pad();

        public native PoolingLayer pad(opencv_core.Size size);

        @opencv_core.Str
        public native BytePointer padMode();

        public native PoolingLayer padMode(BytePointer bytePointer);

        public native int pad_b();

        public native PoolingLayer pad_b(int i);

        public native int pad_l();

        public native PoolingLayer pad_l(int i);

        public native int pad_r();

        public native PoolingLayer pad_r(int i);

        public native int pad_t();

        public native PoolingLayer pad_t(int i);

        @ByRef
        public native opencv_core.Size pooledSize();

        public native PoolingLayer pooledSize(opencv_core.Size size);

        @Name({"type"})
        public native int poolingType();

        public native PoolingLayer poolingType(int i);

        public native int psRoiOutChannels();

        public native PoolingLayer psRoiOutChannels(int i);

        public native float spatialScale();

        public native PoolingLayer spatialScale(float f);

        @ByRef
        public native opencv_core.Size stride();

        public native PoolingLayer stride(opencv_core.Size size);

        static {
            Loader.load();
        }

        public PoolingLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    @NoOffset
    public static class SoftmaxLayer extends Layer {
        @opencv_core.Ptr
        public static native SoftmaxLayer create(@ByRef @Const LayerParams layerParams);

        public native SoftmaxLayer logSoftMax(boolean z);

        @Cast({"bool"})
        public native boolean logSoftMax();

        static {
            Loader.load();
        }

        public SoftmaxLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    @NoOffset
    public static class InnerProductLayer extends Layer {
        @opencv_core.Ptr
        public static native InnerProductLayer create(@ByRef @Const LayerParams layerParams);

        public native int axis();

        public native InnerProductLayer axis(int i);

        static {
            Loader.load();
        }

        public InnerProductLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    @NoOffset
    public static class MVNLayer extends Layer {
        @opencv_core.Ptr
        public static native MVNLayer create(@ByRef @Const LayerParams layerParams);

        public native MVNLayer acrossChannels(boolean z);

        @Cast({"bool"})
        public native boolean acrossChannels();

        public native float eps();

        public native MVNLayer eps(float f);

        public native MVNLayer normVariance(boolean z);

        @Cast({"bool"})
        public native boolean normVariance();

        static {
            Loader.load();
        }

        public MVNLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    @NoOffset
    public static class ReshapeLayer extends Layer {
        @opencv_core.Ptr
        public static native ReshapeLayer create(@ByRef @Const LayerParams layerParams);

        @ByRef
        @StdVector
        public native IntPointer newShapeDesc();

        public native ReshapeLayer newShapeDesc(IntPointer intPointer);

        @ByRef
        public native opencv_core.Range newShapeRange();

        public native ReshapeLayer newShapeRange(opencv_core.Range range);

        static {
            Loader.load();
        }

        public ReshapeLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    public static class FlattenLayer extends Layer {
        @opencv_core.Ptr
        public static native FlattenLayer create(@ByRef @Const LayerParams layerParams);

        static {
            Loader.load();
        }

        public FlattenLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    @NoOffset
    public static class ConcatLayer extends Layer {
        @opencv_core.Ptr
        public static native ConcatLayer create(@ByRef @Const LayerParams layerParams);

        public native int axis();

        public native ConcatLayer axis(int i);

        public native ConcatLayer padding(boolean z);

        @Cast({"bool"})
        public native boolean padding();

        static {
            Loader.load();
        }

        public ConcatLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    @NoOffset
    public static class SplitLayer extends Layer {
        @opencv_core.Ptr
        public static native SplitLayer create(@ByRef @Const LayerParams layerParams);

        public native int outputsCount();

        public native SplitLayer outputsCount(int i);

        static {
            Loader.load();
        }

        public SplitLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    @NoOffset
    public static class SliceLayer extends Layer {
        @opencv_core.Ptr
        public static native SliceLayer create(@ByRef @Const LayerParams layerParams);

        public native int axis();

        public native SliceLayer axis(int i);

        @ByRef
        public native RangeVectorVector sliceRanges();

        public native SliceLayer sliceRanges(RangeVectorVector rangeVectorVector);

        static {
            Loader.load();
        }

        public SliceLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    public static class PermuteLayer extends Layer {
        @opencv_core.Ptr
        public static native PermuteLayer create(@ByRef @Const LayerParams layerParams);

        static {
            Loader.load();
        }

        public PermuteLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    @NoOffset
    public static class ShuffleChannelLayer extends Layer {
        @opencv_core.Ptr
        public static native Layer create(@ByRef @Const LayerParams layerParams);

        public native int group();

        public native ShuffleChannelLayer group(int i);

        static {
            Loader.load();
        }

        public ShuffleChannelLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    public static class PaddingLayer extends Layer {
        @opencv_core.Ptr
        public static native PaddingLayer create(@ByRef @Const LayerParams layerParams);

        static {
            Loader.load();
        }

        public PaddingLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    public static class ActivationLayer extends Layer {
        public native void forwardSlice(@Const FloatBuffer floatBuffer, FloatBuffer floatBuffer2, int i, @Cast({"size_t"}) long j, int i2, int i3);

        public native void forwardSlice(@Const FloatPointer floatPointer, FloatPointer floatPointer2, int i, @Cast({"size_t"}) long j, int i2, int i3);

        public native void forwardSlice(@Const float[] fArr, float[] fArr2, int i, @Cast({"size_t"}) long j, int i2, int i3);

        static {
            Loader.load();
        }

        public ActivationLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    @NoOffset
    public static class ReLULayer extends ActivationLayer {
        @opencv_core.Ptr
        public static native ReLULayer create(@ByRef @Const LayerParams layerParams);

        public native float negativeSlope();

        public native ReLULayer negativeSlope(float f);

        static {
            Loader.load();
        }

        public ReLULayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    @NoOffset
    public static class ReLU6Layer extends ActivationLayer {
        @opencv_core.Ptr
        public static native ReLU6Layer create(@ByRef @Const LayerParams layerParams);

        public native float maxValue();

        public native ReLU6Layer maxValue(float f);

        public native float minValue();

        public native ReLU6Layer minValue(float f);

        static {
            Loader.load();
        }

        public ReLU6Layer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    public static class ChannelsPReLULayer extends ActivationLayer {
        @opencv_core.Ptr
        public static native Layer create(@ByRef @Const LayerParams layerParams);

        static {
            Loader.load();
        }

        public ChannelsPReLULayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    public static class ELULayer extends ActivationLayer {
        @opencv_core.Ptr
        public static native ELULayer create(@ByRef @Const LayerParams layerParams);

        static {
            Loader.load();
        }

        public ELULayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    public static class TanHLayer extends ActivationLayer {
        @opencv_core.Ptr
        public static native TanHLayer create(@ByRef @Const LayerParams layerParams);

        static {
            Loader.load();
        }

        public TanHLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    public static class SigmoidLayer extends ActivationLayer {
        @opencv_core.Ptr
        public static native SigmoidLayer create(@ByRef @Const LayerParams layerParams);

        static {
            Loader.load();
        }

        public SigmoidLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    public static class BNLLLayer extends ActivationLayer {
        @opencv_core.Ptr
        public static native BNLLLayer create(@ByRef @Const LayerParams layerParams);

        static {
            Loader.load();
        }

        public BNLLLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    public static class AbsLayer extends ActivationLayer {
        @opencv_core.Ptr
        public static native AbsLayer create(@ByRef @Const LayerParams layerParams);

        static {
            Loader.load();
        }

        public AbsLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    @NoOffset
    public static class PowerLayer extends ActivationLayer {
        @opencv_core.Ptr
        public static native PowerLayer create(@ByRef @Const LayerParams layerParams);

        public native float power();

        public native PowerLayer power(float f);

        public native float scale();

        public native PowerLayer scale(float f);

        public native float shift();

        public native PowerLayer shift(float f);

        static {
            Loader.load();
        }

        public PowerLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    @NoOffset
    public static class CropLayer extends Layer {
        @opencv_core.Ptr
        public static native CropLayer create(@ByRef @Const LayerParams layerParams);

        @StdVector
        public native IntPointer offset();

        public native CropLayer offset(IntPointer intPointer);

        public native int startAxis();

        public native CropLayer startAxis(int i);

        static {
            Loader.load();
        }

        public CropLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    public static class EltwiseLayer extends Layer {
        @opencv_core.Ptr
        public static native EltwiseLayer create(@ByRef @Const LayerParams layerParams);

        static {
            Loader.load();
        }

        public EltwiseLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    @NoOffset
    public static class BatchNormLayer extends ActivationLayer {
        @opencv_core.Ptr
        public static native BatchNormLayer create(@ByRef @Const LayerParams layerParams);

        public native float epsilon();

        public native BatchNormLayer epsilon(float f);

        public native BatchNormLayer hasBias(boolean z);

        @Cast({"bool"})
        public native boolean hasBias();

        public native BatchNormLayer hasWeights(boolean z);

        @Cast({"bool"})
        public native boolean hasWeights();

        static {
            Loader.load();
        }

        public BatchNormLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    @NoOffset
    public static class MaxUnpoolLayer extends Layer {
        @opencv_core.Ptr
        public static native MaxUnpoolLayer create(@ByRef @Const LayerParams layerParams);

        @ByRef
        public native opencv_core.Size poolKernel();

        public native MaxUnpoolLayer poolKernel(opencv_core.Size size);

        @ByRef
        public native opencv_core.Size poolPad();

        public native MaxUnpoolLayer poolPad(opencv_core.Size size);

        @ByRef
        public native opencv_core.Size poolStride();

        public native MaxUnpoolLayer poolStride(opencv_core.Size size);

        static {
            Loader.load();
        }

        public MaxUnpoolLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    @NoOffset
    public static class ScaleLayer extends Layer {
        @opencv_core.Ptr
        public static native ScaleLayer create(@ByRef @Const LayerParams layerParams);

        public native int axis();

        public native ScaleLayer axis(int i);

        public native ScaleLayer hasBias(boolean z);

        @Cast({"bool"})
        public native boolean hasBias();

        static {
            Loader.load();
        }

        public ScaleLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    public static class ShiftLayer extends Layer {
        @opencv_core.Ptr
        public static native Layer create(@ByRef @Const LayerParams layerParams);

        static {
            Loader.load();
        }

        public ShiftLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    public static class PriorBoxLayer extends Layer {
        @opencv_core.Ptr
        public static native PriorBoxLayer create(@ByRef @Const LayerParams layerParams);

        static {
            Loader.load();
        }

        public PriorBoxLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    public static class ReorgLayer extends Layer {
        @opencv_core.Ptr
        public static native ReorgLayer create(@ByRef @Const LayerParams layerParams);

        static {
            Loader.load();
        }

        public ReorgLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    public static class RegionLayer extends Layer {
        @opencv_core.Ptr
        public static native RegionLayer create(@ByRef @Const LayerParams layerParams);

        static {
            Loader.load();
        }

        public RegionLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    public static class DetectionOutputLayer extends Layer {
        @opencv_core.Ptr
        public static native DetectionOutputLayer create(@ByRef @Const LayerParams layerParams);

        static {
            Loader.load();
        }

        public DetectionOutputLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    @NoOffset
    public static class NormalizeBBoxLayer extends Layer {
        @opencv_core.Ptr
        public static native NormalizeBBoxLayer create(@ByRef @Const LayerParams layerParams);

        public native NormalizeBBoxLayer acrossSpatial(boolean z);

        @Deprecated
        @Cast({"bool"})
        public native boolean acrossSpatial();

        public native float epsilon();

        public native NormalizeBBoxLayer epsilon(float f);

        public native float pnorm();

        public native NormalizeBBoxLayer pnorm(float f);

        static {
            Loader.load();
        }

        public NormalizeBBoxLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    public static class ResizeLayer extends Layer {
        @opencv_core.Ptr
        public static native ResizeLayer create(@ByRef @Const LayerParams layerParams);

        static {
            Loader.load();
        }

        public ResizeLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    public static class InterpLayer extends Layer {
        @opencv_core.Ptr
        public static native Layer create(@ByRef @Const LayerParams layerParams);

        static {
            Loader.load();
        }

        public InterpLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    public static class ProposalLayer extends Layer {
        @opencv_core.Ptr
        public static native ProposalLayer create(@ByRef @Const LayerParams layerParams);

        static {
            Loader.load();
        }

        public ProposalLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    public static class CropAndResizeLayer extends Layer {
        @opencv_core.Ptr
        public static native Layer create(@ByRef @Const LayerParams layerParams);

        static {
            Loader.load();
        }

        public CropAndResizeLayer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    @NoOffset
    public static class LayerParams extends Dict {
        private native void allocate();

        private native void allocateArray(long j);

        @ByRef
        public native opencv_core.MatVector blobs();

        public native LayerParams blobs(opencv_core.MatVector matVector);

        @opencv_core.Str
        public native BytePointer name();

        public native LayerParams name(BytePointer bytePointer);

        @opencv_core.Str
        public native BytePointer type();

        public native LayerParams type(BytePointer bytePointer);

        static {
            Loader.load();
        }

        public LayerParams() {
            super((Pointer) null);
            allocate();
        }

        public LayerParams(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public LayerParams(Pointer p) {
            super(p);
        }

        public LayerParams position(long position) {
            return (LayerParams) super.position(position);
        }
    }

    @Namespace("cv::dnn")
    @NoOffset
    public static class BackendNode extends Pointer {
        private native void allocate(int i);

        public native int backendId();

        public native BackendNode backendId(int i);

        static {
            Loader.load();
        }

        public BackendNode(Pointer p) {
            super(p);
        }

        public BackendNode(int backendId) {
            super((Pointer) null);
            allocate(backendId);
        }
    }

    @Namespace("cv::dnn")
    @NoOffset
    public static class BackendWrapper extends Pointer {
        public native int backendId();

        public native BackendWrapper backendId(int i);

        public native void copyToHost();

        public native void setHostDirty();

        public native int targetId();

        public native BackendWrapper targetId(int i);

        static {
            Loader.load();
        }

        public BackendWrapper(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::dnn")
    @NoOffset
    public static class Layer extends opencv_core.Algorithm {
        private native void allocate();

        private native void allocate(@ByRef @Const LayerParams layerParams);

        private native void allocateArray(long j);

        public native void applyHalideScheduler(@opencv_core.Ptr BackendNode backendNode, @ByRef @Const MatPointerVector matPointerVector, @ByRef @Const opencv_core.MatVector matVector, int i);

        @ByRef
        public native opencv_core.MatVector blobs();

        public native Layer blobs(opencv_core.MatVector matVector);

        @Deprecated
        @ByVal
        public native opencv_core.MatVector finalize(@ByRef @Const opencv_core.MatVector matVector);

        public native void finalize(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2);

        public native void finalize(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2);

        public native void finalize(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2);

        @Deprecated
        public native void finalize(@ByRef @Const MatPointerVector matPointerVector, @ByRef opencv_core.MatVector matVector);

        public native void forward(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByVal opencv_core.GpuMatVector gpuMatVector3);

        public native void forward(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.MatVector matVector3);

        public native void forward(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @ByVal opencv_core.UMatVector uMatVector3);

        @Deprecated
        public native void forward(@ByRef MatPointerVector matPointerVector, @ByRef opencv_core.MatVector matVector, @ByRef opencv_core.MatVector matVector2);

        public native void forward_fallback(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByVal opencv_core.GpuMatVector gpuMatVector2, @ByVal opencv_core.GpuMatVector gpuMatVector3);

        public native void forward_fallback(@ByVal opencv_core.MatVector matVector, @ByVal opencv_core.MatVector matVector2, @ByVal opencv_core.MatVector matVector3);

        public native void forward_fallback(@ByVal opencv_core.UMatVector uMatVector, @ByVal opencv_core.UMatVector uMatVector2, @ByVal opencv_core.UMatVector uMatVector3);

        @Cast({"int64"})
        public native long getFLOPS(@ByRef @Const MatShapeVector matShapeVector, @ByRef @Const MatShapeVector matShapeVector2);

        @Cast({"bool"})
        public native boolean getMemoryShapes(@ByRef @Const MatShapeVector matShapeVector, int i, @ByRef MatShapeVector matShapeVector2, @ByRef MatShapeVector matShapeVector3);

        public native void getScaleShift(@ByRef opencv_core.Mat mat, @ByRef opencv_core.Mat mat2);

        public native int inputNameToIndex(@opencv_core.Str String str);

        public native int inputNameToIndex(@opencv_core.Str BytePointer bytePointer);

        @opencv_core.Str
        public native BytePointer name();

        public native Layer name(BytePointer bytePointer);

        public native int outputNameToIndex(@opencv_core.Str String str);

        public native int outputNameToIndex(@opencv_core.Str BytePointer bytePointer);

        public native int preferableTarget();

        public native Layer preferableTarget(int i);

        @Deprecated
        public native void run(@ByRef @Const opencv_core.MatVector matVector, @ByRef opencv_core.MatVector matVector2, @ByRef opencv_core.MatVector matVector3);

        @Cast({"bool"})
        public native boolean setActivation(@opencv_core.Ptr ActivationLayer activationLayer);

        public native void setParamsFrom(@ByRef @Const LayerParams layerParams);

        @Cast({"bool"})
        public native boolean supportBackend(int i);

        @opencv_core.Ptr
        public native BackendNode tryAttach(@opencv_core.Ptr BackendNode backendNode);

        @Cast({"bool"})
        public native boolean tryFuse(@opencv_core.Ptr Layer layer);

        @opencv_core.Str
        public native BytePointer type();

        public native Layer type(BytePointer bytePointer);

        public native void unsetAttached();

        static {
            Loader.load();
        }

        public Layer(Pointer p) {
            super(p);
        }

        public Layer(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Layer position(long position) {
            return (Layer) super.position(position);
        }

        public Layer() {
            super((Pointer) null);
            allocate();
        }

        public Layer(@ByRef @Const LayerParams params) {
            super((Pointer) null);
            allocate(params);
        }
    }

    @Namespace("cv::dnn")
    @NoOffset
    public static class Net extends Pointer {
        private native void allocate();

        private native void allocateArray(long j);

        @ByVal
        public static native Net readFromModelOptimizer(@opencv_core.Str String str, @opencv_core.Str String str2);

        @ByVal
        public static native Net readFromModelOptimizer(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

        public native int addLayer(@opencv_core.Str String str, @opencv_core.Str String str2, @ByRef LayerParams layerParams);

        public native int addLayer(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @ByRef LayerParams layerParams);

        public native int addLayerToPrev(@opencv_core.Str String str, @opencv_core.Str String str2, @ByRef LayerParams layerParams);

        public native int addLayerToPrev(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @ByRef LayerParams layerParams);

        public native void connect(int i, int i2, int i3, int i4);

        public native void connect(@opencv_core.Str String str, @opencv_core.Str String str2);

        public native void connect(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

        @Cast({"bool"})
        public native boolean empty();

        public native void enableFusion(@Cast({"bool"}) boolean z);

        @ByVal
        public native opencv_core.Mat forward();

        @ByVal
        public native opencv_core.Mat forward(@opencv_core.Str String str);

        @ByVal
        public native opencv_core.Mat forward(@opencv_core.Str BytePointer bytePointer);

        public native void forward(@ByVal opencv_core.GpuMatVector gpuMatVector);

        public native void forward(@ByVal opencv_core.GpuMatVector gpuMatVector, @opencv_core.Str String str);

        public native void forward(@ByVal opencv_core.GpuMatVector gpuMatVector, @opencv_core.Str BytePointer bytePointer);

        public native void forward(@ByVal opencv_core.GpuMatVector gpuMatVector, @ByRef @Const opencv_core.StringVector stringVector);

        public native void forward(@ByVal opencv_core.MatVector matVector);

        public native void forward(@ByVal opencv_core.MatVector matVector, @opencv_core.Str String str);

        public native void forward(@ByVal opencv_core.MatVector matVector, @opencv_core.Str BytePointer bytePointer);

        public native void forward(@ByVal opencv_core.MatVector matVector, @ByRef @Const opencv_core.StringVector stringVector);

        public native void forward(@ByVal opencv_core.UMatVector uMatVector);

        public native void forward(@ByVal opencv_core.UMatVector uMatVector, @opencv_core.Str String str);

        public native void forward(@ByVal opencv_core.UMatVector uMatVector, @opencv_core.Str BytePointer bytePointer);

        public native void forward(@ByVal opencv_core.UMatVector uMatVector, @ByRef @Const opencv_core.StringVector stringVector);

        @Name({"forward"})
        public native void forwardAndRetrieve(@ByRef opencv_core.MatVectorVector matVectorVector, @ByRef @Const opencv_core.StringVector stringVector);

        @Cast({"int64"})
        public native long getFLOPS(int i, @ByRef @Const @StdVector IntPointer intPointer);

        @Cast({"int64"})
        public native long getFLOPS(int i, @ByRef @Const MatShapeVector matShapeVector);

        @Cast({"int64"})
        public native long getFLOPS(@ByRef @Const @StdVector IntPointer intPointer);

        @Cast({"int64"})
        public native long getFLOPS(@ByRef @Const MatShapeVector matShapeVector);

        @opencv_core.Ptr
        public native Layer getLayer(@Cast({"cv::dnn::Net::LayerId*"}) @ByVal DictValue dictValue);

        public native int getLayerId(@opencv_core.Str String str);

        public native int getLayerId(@opencv_core.Str BytePointer bytePointer);

        @ByVal
        public native opencv_core.StringVector getLayerNames();

        public native void getLayerShapes(@ByRef @Const @StdVector IntPointer intPointer, int i, @ByRef MatShapeVector matShapeVector, @ByRef MatShapeVector matShapeVector2);

        public native void getLayerShapes(@ByRef @Const MatShapeVector matShapeVector, int i, @ByRef MatShapeVector matShapeVector2, @ByRef MatShapeVector matShapeVector3);

        public native void getLayerTypes(@ByRef opencv_core.StringVector stringVector);

        public native int getLayersCount(@opencv_core.Str String str);

        public native int getLayersCount(@opencv_core.Str BytePointer bytePointer);

        public native void getLayersShapes(@ByRef @Const @StdVector IntPointer intPointer, @StdVector IntBuffer intBuffer, @ByRef MatShapeVectorVector matShapeVectorVector, @ByRef MatShapeVectorVector matShapeVectorVector2);

        public native void getLayersShapes(@ByRef @Const @StdVector IntPointer intPointer, @StdVector IntPointer intPointer2, @ByRef MatShapeVectorVector matShapeVectorVector, @ByRef MatShapeVectorVector matShapeVectorVector2);

        public native void getLayersShapes(@ByRef @Const @StdVector IntPointer intPointer, @StdVector int[] iArr, @ByRef MatShapeVectorVector matShapeVectorVector, @ByRef MatShapeVectorVector matShapeVectorVector2);

        public native void getLayersShapes(@ByRef @Const MatShapeVector matShapeVector, @StdVector IntBuffer intBuffer, @ByRef MatShapeVectorVector matShapeVectorVector, @ByRef MatShapeVectorVector matShapeVectorVector2);

        public native void getLayersShapes(@ByRef @Const MatShapeVector matShapeVector, @StdVector IntPointer intPointer, @ByRef MatShapeVectorVector matShapeVectorVector, @ByRef MatShapeVectorVector matShapeVectorVector2);

        public native void getLayersShapes(@ByRef @Const MatShapeVector matShapeVector, @StdVector int[] iArr, @ByRef MatShapeVectorVector matShapeVectorVector, @ByRef MatShapeVectorVector matShapeVectorVector2);

        public native void getMemoryConsumption(int i, @ByRef @Const @StdVector IntPointer intPointer, @ByRef @Cast({"size_t*"}) SizeTPointer sizeTPointer, @ByRef @Cast({"size_t*"}) SizeTPointer sizeTPointer2);

        public native void getMemoryConsumption(int i, @ByRef @Const MatShapeVector matShapeVector, @ByRef @Cast({"size_t*"}) SizeTPointer sizeTPointer, @ByRef @Cast({"size_t*"}) SizeTPointer sizeTPointer2);

        public native void getMemoryConsumption(@ByRef @Const @StdVector IntPointer intPointer, @StdVector IntBuffer intBuffer, @Cast({"size_t*"}) @StdVector SizeTPointer sizeTPointer, @Cast({"size_t*"}) @StdVector SizeTPointer sizeTPointer2);

        public native void getMemoryConsumption(@ByRef @Const @StdVector IntPointer intPointer, @StdVector IntPointer intPointer2, @Cast({"size_t*"}) @StdVector SizeTPointer sizeTPointer, @Cast({"size_t*"}) @StdVector SizeTPointer sizeTPointer2);

        public native void getMemoryConsumption(@ByRef @Const @StdVector IntPointer intPointer, @ByRef @Cast({"size_t*"}) SizeTPointer sizeTPointer, @ByRef @Cast({"size_t*"}) SizeTPointer sizeTPointer2);

        public native void getMemoryConsumption(@ByRef @Const @StdVector IntPointer intPointer, @StdVector int[] iArr, @Cast({"size_t*"}) @StdVector SizeTPointer sizeTPointer, @Cast({"size_t*"}) @StdVector SizeTPointer sizeTPointer2);

        public native void getMemoryConsumption(@ByRef @Const MatShapeVector matShapeVector, @StdVector IntBuffer intBuffer, @Cast({"size_t*"}) @StdVector SizeTPointer sizeTPointer, @Cast({"size_t*"}) @StdVector SizeTPointer sizeTPointer2);

        public native void getMemoryConsumption(@ByRef @Const MatShapeVector matShapeVector, @StdVector IntPointer intPointer, @Cast({"size_t*"}) @StdVector SizeTPointer sizeTPointer, @Cast({"size_t*"}) @StdVector SizeTPointer sizeTPointer2);

        public native void getMemoryConsumption(@ByRef @Const MatShapeVector matShapeVector, @ByRef @Cast({"size_t*"}) SizeTPointer sizeTPointer, @ByRef @Cast({"size_t*"}) SizeTPointer sizeTPointer2);

        public native void getMemoryConsumption(@ByRef @Const MatShapeVector matShapeVector, @StdVector int[] iArr, @Cast({"size_t*"}) @StdVector SizeTPointer sizeTPointer, @Cast({"size_t*"}) @StdVector SizeTPointer sizeTPointer2);

        @ByVal
        public native opencv_core.Mat getParam(@Cast({"cv::dnn::Net::LayerId*"}) @ByVal DictValue dictValue);

        @ByVal
        public native opencv_core.Mat getParam(@Cast({"cv::dnn::Net::LayerId*"}) @ByVal DictValue dictValue, int i);

        @Cast({"int64"})
        public native long getPerfProfile(@StdVector DoubleBuffer doubleBuffer);

        @Cast({"int64"})
        public native long getPerfProfile(@StdVector DoublePointer doublePointer);

        @Cast({"int64"})
        public native long getPerfProfile(@StdVector double[] dArr);

        @StdVector
        public native IntPointer getUnconnectedOutLayers();

        @ByVal
        public native opencv_core.StringVector getUnconnectedOutLayersNames();

        public native void setHalideScheduler(@opencv_core.Str String str);

        public native void setHalideScheduler(@opencv_core.Str BytePointer bytePointer);

        public native void setInput(@ByVal opencv_core.GpuMat gpuMat);

        public native void setInput(@ByVal opencv_core.GpuMat gpuMat, @opencv_core.Str String str, double d, @ByRef(nullValue = "cv::Scalar()") @Const opencv_core.Scalar scalar);

        public native void setInput(@ByVal opencv_core.GpuMat gpuMat, @opencv_core.Str BytePointer bytePointer, double d, @ByRef(nullValue = "cv::Scalar()") @Const opencv_core.Scalar scalar);

        public native void setInput(@ByVal opencv_core.Mat mat);

        public native void setInput(@ByVal opencv_core.Mat mat, @opencv_core.Str String str, double d, @ByRef(nullValue = "cv::Scalar()") @Const opencv_core.Scalar scalar);

        public native void setInput(@ByVal opencv_core.Mat mat, @opencv_core.Str BytePointer bytePointer, double d, @ByRef(nullValue = "cv::Scalar()") @Const opencv_core.Scalar scalar);

        public native void setInput(@ByVal opencv_core.UMat uMat);

        public native void setInput(@ByVal opencv_core.UMat uMat, @opencv_core.Str String str, double d, @ByRef(nullValue = "cv::Scalar()") @Const opencv_core.Scalar scalar);

        public native void setInput(@ByVal opencv_core.UMat uMat, @opencv_core.Str BytePointer bytePointer, double d, @ByRef(nullValue = "cv::Scalar()") @Const opencv_core.Scalar scalar);

        public native void setInputsNames(@ByRef @Const opencv_core.StringVector stringVector);

        public native void setParam(@Cast({"cv::dnn::Net::LayerId*"}) @ByVal DictValue dictValue, int i, @ByRef @Const opencv_core.Mat mat);

        public native void setPreferableBackend(int i);

        public native void setPreferableTarget(int i);

        static {
            Loader.load();
        }

        public Net(Pointer p) {
            super(p);
        }

        public Net(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public Net position(long position) {
            return (Net) super.position(position);
        }

        public Net() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::dnn")
    public static class LayerFactory extends Pointer {
        @opencv_core.Ptr
        public static native Layer createLayerInstance(@opencv_core.Str String str, @ByRef LayerParams layerParams);

        @opencv_core.Ptr
        public static native Layer createLayerInstance(@opencv_core.Str BytePointer bytePointer, @ByRef LayerParams layerParams);

        public static native void registerLayer(@opencv_core.Str String str, Constructor constructor);

        public static native void registerLayer(@opencv_core.Str BytePointer bytePointer, Constructor constructor);

        public static native void unregisterLayer(@opencv_core.Str String str);

        public static native void unregisterLayer(@opencv_core.Str BytePointer bytePointer);

        static {
            Loader.load();
        }

        public LayerFactory(Pointer p) {
            super(p);
        }

        @Convention(extern = "C++", value = "")
        public static class Constructor extends FunctionPointer {
            private native void allocate();

            @opencv_core.Ptr
            public native Layer call(@ByRef LayerParams layerParams);

            static {
                Loader.load();
            }

            public Constructor(Pointer p) {
                super(p);
            }

            protected Constructor() {
                allocate();
            }
        }
    }

    @Namespace("cv::dnn")
    public static class _Range extends opencv_core.Range {
        private native void allocate(int i);

        private native void allocate(int i, int i2);

        private native void allocate(@ByRef @Const opencv_core.Range range);

        static {
            Loader.load();
        }

        public _Range(Pointer p) {
            super(p);
        }

        public _Range(@ByRef @Const opencv_core.Range r) {
            super((Pointer) null);
            allocate(r);
        }

        public _Range(int start_, int size_) {
            super((Pointer) null);
            allocate(start_, size_);
        }

        public _Range(int start_) {
            super((Pointer) null);
            allocate(start_);
        }
    }
}
