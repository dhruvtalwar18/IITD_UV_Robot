package org.bytedeco.javacpp;

import java.nio.DoubleBuffer;
import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Const;
import org.bytedeco.javacpp.annotation.Name;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.annotation.StdVector;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.presets.opencv_core;

public class opencv_cudaobjdetect extends org.bytedeco.javacpp.presets.opencv_cudaobjdetect {
    static {
        Loader.load();
    }

    @Namespace("cv::cuda")
    public static class HOG extends opencv_core.Algorithm {
        @opencv_core.Ptr
        public static native HOG create();

        @opencv_core.Ptr
        public static native HOG create(@ByVal(nullValue = "cv::Size(64, 128)") opencv_core.Size size, @ByVal(nullValue = "cv::Size(16, 16)") opencv_core.Size size2, @ByVal(nullValue = "cv::Size(8, 8)") opencv_core.Size size3, @ByVal(nullValue = "cv::Size(8, 8)") opencv_core.Size size4, int i);

        public native void compute(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void compute(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void compute(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void compute(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void compute(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void compute(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.PointVector pointVector);

        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.PointVector pointVector, @StdVector DoubleBuffer doubleBuffer);

        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.PointVector pointVector, @StdVector DoublePointer doublePointer);

        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.PointVector pointVector, @StdVector double[] dArr);

        public native void detect(@ByVal opencv_core.Mat mat, @ByRef opencv_core.PointVector pointVector);

        public native void detect(@ByVal opencv_core.Mat mat, @ByRef opencv_core.PointVector pointVector, @StdVector DoubleBuffer doubleBuffer);

        public native void detect(@ByVal opencv_core.Mat mat, @ByRef opencv_core.PointVector pointVector, @StdVector DoublePointer doublePointer);

        public native void detect(@ByVal opencv_core.Mat mat, @ByRef opencv_core.PointVector pointVector, @StdVector double[] dArr);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.PointVector pointVector);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.PointVector pointVector, @StdVector DoubleBuffer doubleBuffer);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.PointVector pointVector, @StdVector DoublePointer doublePointer);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.PointVector pointVector, @StdVector double[] dArr);

        public native void detectMultiScale(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector);

        public native void detectMultiScale(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector DoubleBuffer doubleBuffer);

        public native void detectMultiScale(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector DoublePointer doublePointer);

        public native void detectMultiScale(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector double[] dArr);

        public native void detectMultiScale(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector);

        public native void detectMultiScale(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector DoubleBuffer doubleBuffer);

        public native void detectMultiScale(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector DoublePointer doublePointer);

        public native void detectMultiScale(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector double[] dArr);

        public native void detectMultiScale(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector);

        public native void detectMultiScale(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector DoubleBuffer doubleBuffer);

        public native void detectMultiScale(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector DoublePointer doublePointer);

        public native void detectMultiScale(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector double[] dArr);

        @Cast({"size_t"})
        public native long getBlockHistogramSize();

        @ByVal
        public native opencv_core.Mat getDefaultPeopleDetector();

        @Cast({"cv::HOGDescriptor::DescriptorStorageFormat"})
        public native int getDescriptorFormat();

        @Cast({"size_t"})
        public native long getDescriptorSize();

        @Cast({"bool"})
        public native boolean getGammaCorrection();

        public native int getGroupThreshold();

        public native double getHitThreshold();

        public native double getL2HysThreshold();

        public native int getNumLevels();

        public native double getScaleFactor();

        public native double getWinSigma();

        @ByVal
        public native opencv_core.Size getWinStride();

        public native void setDescriptorFormat(@Cast({"cv::HOGDescriptor::DescriptorStorageFormat"}) int i);

        public native void setGammaCorrection(@Cast({"bool"}) boolean z);

        public native void setGroupThreshold(int i);

        public native void setHitThreshold(double d);

        public native void setL2HysThreshold(double d);

        public native void setNumLevels(int i);

        public native void setSVMDetector(@ByVal opencv_core.GpuMat gpuMat);

        public native void setSVMDetector(@ByVal opencv_core.Mat mat);

        public native void setSVMDetector(@ByVal opencv_core.UMat uMat);

        public native void setScaleFactor(double d);

        public native void setWinSigma(double d);

        public native void setWinStride(@ByVal opencv_core.Size size);

        static {
            Loader.load();
        }

        public HOG(Pointer p) {
            super(p);
        }
    }

    @Name({"cv::cuda::CascadeClassifier"})
    public static class CudaCascadeClassifier extends opencv_core.Algorithm {
        @opencv_core.Ptr
        public static native CudaCascadeClassifier create(@opencv_core.Str String str);

        @opencv_core.Ptr
        public static native CudaCascadeClassifier create(@opencv_core.Str BytePointer bytePointer);

        @opencv_core.Ptr
        public static native CudaCascadeClassifier create(@ByRef @Const opencv_core.FileStorage fileStorage);

        public native void convert(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector);

        public native void convert(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector);

        public native void convert(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector);

        public native void detectMultiScale(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void detectMultiScale(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void detectMultiScale(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void detectMultiScale(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        public native void detectMultiScale(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void detectMultiScale(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef(nullValue = "cv::cuda::Stream::Null()") opencv_core.Stream stream);

        @ByVal
        public native opencv_core.Size getClassifierSize();

        @Cast({"bool"})
        public native boolean getFindLargestObject();

        public native int getMaxNumObjects();

        @ByVal
        public native opencv_core.Size getMaxObjectSize();

        public native int getMinNeighbors();

        @ByVal
        public native opencv_core.Size getMinObjectSize();

        public native double getScaleFactor();

        public native void setFindLargestObject(@Cast({"bool"}) boolean z);

        public native void setMaxNumObjects(int i);

        public native void setMaxObjectSize(@ByVal opencv_core.Size size);

        public native void setMinNeighbors(int i);

        public native void setMinObjectSize(@ByVal opencv_core.Size size);

        public native void setScaleFactor(double d);

        static {
            Loader.load();
        }

        public CudaCascadeClassifier(Pointer p) {
            super(p);
        }
    }
}
