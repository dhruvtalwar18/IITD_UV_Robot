package org.bytedeco.javacpp;

import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.presets.opencv_core;

public class opencv_plot extends org.bytedeco.javacpp.presets.opencv_plot {
    static {
        Loader.load();
    }

    @Namespace("cv::plot")
    public static class Plot2d extends opencv_core.Algorithm {
        @opencv_core.Ptr
        public static native Plot2d create(@ByVal opencv_core.GpuMat gpuMat);

        @opencv_core.Ptr
        public static native Plot2d create(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        @opencv_core.Ptr
        public static native Plot2d create(@ByVal opencv_core.Mat mat);

        @opencv_core.Ptr
        public static native Plot2d create(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        @opencv_core.Ptr
        public static native Plot2d create(@ByVal opencv_core.UMat uMat);

        @opencv_core.Ptr
        public static native Plot2d create(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void render(@ByVal opencv_core.GpuMat gpuMat);

        public native void render(@ByVal opencv_core.Mat mat);

        public native void render(@ByVal opencv_core.UMat uMat);

        public native void setGridLinesNumber(int i);

        public native void setInvertOrientation(@Cast({"bool"}) boolean z);

        public native void setMaxX(double d);

        public native void setMaxY(double d);

        public native void setMinX(double d);

        public native void setMinY(double d);

        public native void setNeedPlotLine(@Cast({"bool"}) boolean z);

        public native void setPlotAxisColor(@ByVal opencv_core.Scalar scalar);

        public native void setPlotBackgroundColor(@ByVal opencv_core.Scalar scalar);

        public native void setPlotGridColor(@ByVal opencv_core.Scalar scalar);

        public native void setPlotLineColor(@ByVal opencv_core.Scalar scalar);

        public native void setPlotLineWidth(int i);

        public native void setPlotSize(int i, int i2);

        public native void setPlotTextColor(@ByVal opencv_core.Scalar scalar);

        public native void setPointIdxToPrint(int i);

        public native void setShowGrid(@Cast({"bool"}) boolean z);

        public native void setShowText(@Cast({"bool"}) boolean z);

        static {
            Loader.load();
        }

        public Plot2d(Pointer p) {
            super(p);
        }
    }
}
