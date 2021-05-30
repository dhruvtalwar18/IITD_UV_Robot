package org.bytedeco.javacpp;

import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Const;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.annotation.Opaque;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.presets.opencv_core;

public class opencv_shape extends org.bytedeco.javacpp.presets.opencv_shape {
    @Namespace("cv")
    public static native float EMDL1(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

    @Namespace("cv")
    public static native float EMDL1(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

    @Namespace("cv")
    public static native float EMDL1(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

    @Namespace("cv")
    @opencv_core.Ptr
    public static native AffineTransformer createAffineTransformer(@Cast({"bool"}) boolean z);

    @Namespace("cv")
    @opencv_core.Ptr
    public static native HistogramCostExtractor createChiHistogramCostExtractor();

    @Namespace("cv")
    @opencv_core.Ptr
    public static native HistogramCostExtractor createChiHistogramCostExtractor(int i, float f);

    @Namespace("cv")
    @opencv_core.Ptr
    public static native HistogramCostExtractor createEMDHistogramCostExtractor();

    @Namespace("cv")
    @opencv_core.Ptr
    public static native HistogramCostExtractor createEMDHistogramCostExtractor(int i, int i2, float f);

    @Namespace("cv")
    @opencv_core.Ptr
    public static native HistogramCostExtractor createEMDL1HistogramCostExtractor();

    @Namespace("cv")
    @opencv_core.Ptr
    public static native HistogramCostExtractor createEMDL1HistogramCostExtractor(int i, float f);

    @Namespace("cv")
    @opencv_core.Ptr
    public static native HausdorffDistanceExtractor createHausdorffDistanceExtractor();

    @Namespace("cv")
    @opencv_core.Ptr
    public static native HausdorffDistanceExtractor createHausdorffDistanceExtractor(int i, float f);

    @Namespace("cv")
    @opencv_core.Ptr
    public static native HistogramCostExtractor createNormHistogramCostExtractor();

    @Namespace("cv")
    @opencv_core.Ptr
    public static native HistogramCostExtractor createNormHistogramCostExtractor(int i, int i2, float f);

    @Namespace("cv")
    @opencv_core.Ptr
    public static native ShapeContextDistanceExtractor createShapeContextDistanceExtractor();

    @Namespace("cv")
    @opencv_core.Ptr
    public static native ShapeContextDistanceExtractor createShapeContextDistanceExtractor(int i, int i2, float f, float f2, int i3, @opencv_core.Ptr HistogramCostExtractor histogramCostExtractor, @opencv_core.Ptr ShapeTransformer shapeTransformer);

    @Namespace("cv")
    @opencv_core.Ptr
    public static native ThinPlateSplineShapeTransformer createThinPlateSplineShapeTransformer();

    @Namespace("cv")
    @opencv_core.Ptr
    public static native ThinPlateSplineShapeTransformer createThinPlateSplineShapeTransformer(double d);

    static {
        Loader.load();
    }

    @Namespace("cv")
    public static class ShapeTransformer extends opencv_core.Algorithm {
        public native float applyTransformation(@ByVal opencv_core.GpuMat gpuMat);

        public native float applyTransformation(@ByVal opencv_core.GpuMat gpuMat, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.GpuMat gpuMat2);

        public native float applyTransformation(@ByVal opencv_core.Mat mat);

        public native float applyTransformation(@ByVal opencv_core.Mat mat, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.Mat mat2);

        public native float applyTransformation(@ByVal opencv_core.UMat uMat);

        public native float applyTransformation(@ByVal opencv_core.UMat uMat, @ByVal(nullValue = "cv::OutputArray(cv::noArray())") opencv_core.UMat uMat2);

        public native void estimateTransformation(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByRef opencv_core.DMatchVector dMatchVector);

        public native void estimateTransformation(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByRef opencv_core.DMatchVector dMatchVector);

        public native void estimateTransformation(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByRef opencv_core.DMatchVector dMatchVector);

        public native void warpImage(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void warpImage(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, int i2, @ByRef(nullValue = "cv::Scalar()") @Const opencv_core.Scalar scalar);

        public native void warpImage(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void warpImage(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, int i2, @ByRef(nullValue = "cv::Scalar()") @Const opencv_core.Scalar scalar);

        public native void warpImage(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void warpImage(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, int i2, @ByRef(nullValue = "cv::Scalar()") @Const opencv_core.Scalar scalar);

        static {
            Loader.load();
        }

        public ShapeTransformer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class ThinPlateSplineShapeTransformer extends ShapeTransformer {
        public native double getRegularizationParameter();

        public native void setRegularizationParameter(double d);

        static {
            Loader.load();
        }

        public ThinPlateSplineShapeTransformer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class AffineTransformer extends ShapeTransformer {
        @Cast({"bool"})
        public native boolean getFullAffine();

        public native void setFullAffine(@Cast({"bool"}) boolean z);

        static {
            Loader.load();
        }

        public AffineTransformer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class HistogramCostExtractor extends opencv_core.Algorithm {
        public native void buildCostMatrix(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.GpuMat gpuMat3);

        public native void buildCostMatrix(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.Mat mat3);

        public native void buildCostMatrix(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.UMat uMat3);

        public native float getDefaultCost();

        public native int getNDummies();

        public native void setDefaultCost(float f);

        public native void setNDummies(int i);

        static {
            Loader.load();
        }

        public HistogramCostExtractor(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class NormHistogramCostExtractor extends HistogramCostExtractor {
        public native int getNormFlag();

        public native void setNormFlag(int i);

        static {
            Loader.load();
        }

        public NormHistogramCostExtractor(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class EMDHistogramCostExtractor extends HistogramCostExtractor {
        public native int getNormFlag();

        public native void setNormFlag(int i);

        static {
            Loader.load();
        }

        public EMDHistogramCostExtractor(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    @Opaque
    public static class ChiHistogramCostExtractor extends HistogramCostExtractor {
        public ChiHistogramCostExtractor() {
            super((Pointer) null);
        }

        public ChiHistogramCostExtractor(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    @Opaque
    public static class EMDL1HistogramCostExtractor extends HistogramCostExtractor {
        public EMDL1HistogramCostExtractor() {
            super((Pointer) null);
        }

        public EMDL1HistogramCostExtractor(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class ShapeDistanceExtractor extends opencv_core.Algorithm {
        public native float computeDistance(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native float computeDistance(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native float computeDistance(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        static {
            Loader.load();
        }

        public ShapeDistanceExtractor(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class ShapeContextDistanceExtractor extends ShapeDistanceExtractor {
        public native int getAngularBins();

        public native float getBendingEnergyWeight();

        @opencv_core.Ptr
        public native HistogramCostExtractor getCostExtractor();

        public native float getImageAppearanceWeight();

        public native void getImages(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void getImages(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void getImages(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native float getInnerRadius();

        public native int getIterations();

        public native float getOuterRadius();

        public native int getRadialBins();

        @Cast({"bool"})
        public native boolean getRotationInvariant();

        public native float getShapeContextWeight();

        public native float getStdDev();

        @opencv_core.Ptr
        public native ShapeTransformer getTransformAlgorithm();

        public native void setAngularBins(int i);

        public native void setBendingEnergyWeight(float f);

        public native void setCostExtractor(@opencv_core.Ptr HistogramCostExtractor histogramCostExtractor);

        public native void setImageAppearanceWeight(float f);

        public native void setImages(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        public native void setImages(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        public native void setImages(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        public native void setInnerRadius(float f);

        public native void setIterations(int i);

        public native void setOuterRadius(float f);

        public native void setRadialBins(int i);

        public native void setRotationInvariant(@Cast({"bool"}) boolean z);

        public native void setShapeContextWeight(float f);

        public native void setStdDev(float f);

        public native void setTransformAlgorithm(@opencv_core.Ptr ShapeTransformer shapeTransformer);

        static {
            Loader.load();
        }

        public ShapeContextDistanceExtractor(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv")
    public static class HausdorffDistanceExtractor extends ShapeDistanceExtractor {
        public native int getDistanceFlag();

        public native float getRankProportion();

        public native void setDistanceFlag(int i);

        public native void setRankProportion(float f);

        static {
            Loader.load();
        }

        public HausdorffDistanceExtractor(Pointer p) {
            super(p);
        }
    }
}
