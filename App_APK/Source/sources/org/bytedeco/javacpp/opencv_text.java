package org.bytedeco.javacpp;

import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.Arrays;
import org.bytedeco.javacpp.annotation.ByRef;
import org.bytedeco.javacpp.annotation.ByVal;
import org.bytedeco.javacpp.annotation.Cast;
import org.bytedeco.javacpp.annotation.Const;
import org.bytedeco.javacpp.annotation.Index;
import org.bytedeco.javacpp.annotation.MemberGetter;
import org.bytedeco.javacpp.annotation.Name;
import org.bytedeco.javacpp.annotation.Namespace;
import org.bytedeco.javacpp.annotation.NoOffset;
import org.bytedeco.javacpp.annotation.StdString;
import org.bytedeco.javacpp.annotation.StdVector;
import org.bytedeco.javacpp.annotation.ValueSetter;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.presets.opencv_core;
import org.xbill.DNS.TTL;

public class opencv_text extends org.bytedeco.javacpp.presets.opencv_text {
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

    @Namespace("cv::text")
    public static native void MSERsToERStats(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.PointVectorVector pointVectorVector, @ByRef ERStatVectorVector eRStatVectorVector);

    @Namespace("cv::text")
    public static native void MSERsToERStats(@ByVal opencv_core.Mat mat, @ByRef opencv_core.PointVectorVector pointVectorVector, @ByRef ERStatVectorVector eRStatVectorVector);

    @Namespace("cv::text")
    public static native void MSERsToERStats(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.PointVectorVector pointVectorVector, @ByRef ERStatVectorVector eRStatVectorVector);

    @Namespace("cv::text")
    public static native void computeNMChannels(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMatVector gpuMatVector);

    @Namespace("cv::text")
    public static native void computeNMChannels(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMatVector gpuMatVector, int i);

    @Namespace("cv::text")
    public static native void computeNMChannels(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.MatVector matVector);

    @Namespace("cv::text")
    public static native void computeNMChannels(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.MatVector matVector, int i);

    @Namespace("cv::text")
    public static native void computeNMChannels(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.UMatVector uMatVector);

    @Namespace("cv::text")
    public static native void computeNMChannels(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.UMatVector uMatVector, int i);

    @Namespace("cv::text")
    public static native void computeNMChannels(@ByVal opencv_core.Mat mat, @ByVal opencv_core.GpuMatVector gpuMatVector);

    @Namespace("cv::text")
    public static native void computeNMChannels(@ByVal opencv_core.Mat mat, @ByVal opencv_core.GpuMatVector gpuMatVector, int i);

    @Namespace("cv::text")
    public static native void computeNMChannels(@ByVal opencv_core.Mat mat, @ByVal opencv_core.MatVector matVector);

    @Namespace("cv::text")
    public static native void computeNMChannels(@ByVal opencv_core.Mat mat, @ByVal opencv_core.MatVector matVector, int i);

    @Namespace("cv::text")
    public static native void computeNMChannels(@ByVal opencv_core.Mat mat, @ByVal opencv_core.UMatVector uMatVector);

    @Namespace("cv::text")
    public static native void computeNMChannels(@ByVal opencv_core.Mat mat, @ByVal opencv_core.UMatVector uMatVector, int i);

    @Namespace("cv::text")
    public static native void computeNMChannels(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.GpuMatVector gpuMatVector);

    @Namespace("cv::text")
    public static native void computeNMChannels(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.GpuMatVector gpuMatVector, int i);

    @Namespace("cv::text")
    public static native void computeNMChannels(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.MatVector matVector);

    @Namespace("cv::text")
    public static native void computeNMChannels(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.MatVector matVector, int i);

    @Namespace("cv::text")
    public static native void computeNMChannels(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMatVector uMatVector);

    @Namespace("cv::text")
    public static native void computeNMChannels(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMatVector uMatVector, int i);

    @Namespace("cv::text")
    @opencv_core.Ptr
    public static native ERFilter createERFilterNM1(@opencv_core.Str String str);

    @Namespace("cv::text")
    @opencv_core.Ptr
    public static native ERFilter createERFilterNM1(@opencv_core.Str String str, int i, float f, float f2, float f3, @Cast({"bool"}) boolean z, float f4);

    @Namespace("cv::text")
    @opencv_core.Ptr
    public static native ERFilter createERFilterNM1(@opencv_core.Str BytePointer bytePointer);

    @Namespace("cv::text")
    @opencv_core.Ptr
    public static native ERFilter createERFilterNM1(@opencv_core.Str BytePointer bytePointer, int i, float f, float f2, float f3, @Cast({"bool"}) boolean z, float f4);

    @Namespace("cv::text")
    @opencv_core.Ptr
    public static native ERFilter createERFilterNM1(@opencv_core.Ptr ERFilter.Callback callback);

    @Namespace("cv::text")
    @opencv_core.Ptr
    public static native ERFilter createERFilterNM1(@opencv_core.Ptr ERFilter.Callback callback, int i, float f, float f2, float f3, @Cast({"bool"}) boolean z, float f4);

    @Namespace("cv::text")
    @opencv_core.Ptr
    public static native ERFilter createERFilterNM2(@opencv_core.Str String str);

    @Namespace("cv::text")
    @opencv_core.Ptr
    public static native ERFilter createERFilterNM2(@opencv_core.Str String str, float f);

    @Namespace("cv::text")
    @opencv_core.Ptr
    public static native ERFilter createERFilterNM2(@opencv_core.Str BytePointer bytePointer);

    @Namespace("cv::text")
    @opencv_core.Ptr
    public static native ERFilter createERFilterNM2(@opencv_core.Str BytePointer bytePointer, float f);

    @Namespace("cv::text")
    @opencv_core.Ptr
    public static native ERFilter createERFilterNM2(@opencv_core.Ptr ERFilter.Callback callback);

    @Namespace("cv::text")
    @opencv_core.Ptr
    public static native ERFilter createERFilterNM2(@opencv_core.Ptr ERFilter.Callback callback, float f);

    @Namespace("cv::text")
    @ByVal
    public static native opencv_core.Mat createOCRHMMTransitionsTable(@opencv_core.Str String str, @ByRef opencv_core.StringVector stringVector);

    @Namespace("cv::text")
    @ByVal
    public static native opencv_core.Mat createOCRHMMTransitionsTable(@opencv_core.Str BytePointer bytePointer, @ByRef opencv_core.StringVector stringVector);

    @Namespace("cv::text")
    public static native void createOCRHMMTransitionsTable(@ByRef @StdString BytePointer bytePointer, @ByRef StdStringVector stdStringVector, @ByVal opencv_core.GpuMat gpuMat);

    @Namespace("cv::text")
    public static native void createOCRHMMTransitionsTable(@ByRef @StdString BytePointer bytePointer, @ByRef StdStringVector stdStringVector, @ByVal opencv_core.Mat mat);

    @Namespace("cv::text")
    public static native void createOCRHMMTransitionsTable(@ByRef @StdString BytePointer bytePointer, @ByRef StdStringVector stdStringVector, @ByVal opencv_core.UMat uMat);

    @Namespace("cv::text")
    public static native void detectRegions(@ByVal opencv_core.GpuMat gpuMat, @opencv_core.Ptr ERFilter eRFilter, @opencv_core.Ptr ERFilter eRFilter2, @ByRef opencv_core.PointVectorVector pointVectorVector);

    @Namespace("cv::text")
    public static native void detectRegions(@ByVal opencv_core.GpuMat gpuMat, @opencv_core.Ptr ERFilter eRFilter, @opencv_core.Ptr ERFilter eRFilter2, @ByRef opencv_core.RectVector rectVector);

    @Namespace("cv::text")
    public static native void detectRegions(@ByVal opencv_core.GpuMat gpuMat, @opencv_core.Ptr ERFilter eRFilter, @opencv_core.Ptr ERFilter eRFilter2, @ByRef opencv_core.RectVector rectVector, int i, @opencv_core.Str String str, float f);

    @Namespace("cv::text")
    public static native void detectRegions(@ByVal opencv_core.GpuMat gpuMat, @opencv_core.Ptr ERFilter eRFilter, @opencv_core.Ptr ERFilter eRFilter2, @ByRef opencv_core.RectVector rectVector, int i, @opencv_core.Str BytePointer bytePointer, float f);

    @Namespace("cv::text")
    public static native void detectRegions(@ByVal opencv_core.Mat mat, @opencv_core.Ptr ERFilter eRFilter, @opencv_core.Ptr ERFilter eRFilter2, @ByRef opencv_core.PointVectorVector pointVectorVector);

    @Namespace("cv::text")
    public static native void detectRegions(@ByVal opencv_core.Mat mat, @opencv_core.Ptr ERFilter eRFilter, @opencv_core.Ptr ERFilter eRFilter2, @ByRef opencv_core.RectVector rectVector);

    @Namespace("cv::text")
    public static native void detectRegions(@ByVal opencv_core.Mat mat, @opencv_core.Ptr ERFilter eRFilter, @opencv_core.Ptr ERFilter eRFilter2, @ByRef opencv_core.RectVector rectVector, int i, @opencv_core.Str String str, float f);

    @Namespace("cv::text")
    public static native void detectRegions(@ByVal opencv_core.Mat mat, @opencv_core.Ptr ERFilter eRFilter, @opencv_core.Ptr ERFilter eRFilter2, @ByRef opencv_core.RectVector rectVector, int i, @opencv_core.Str BytePointer bytePointer, float f);

    @Namespace("cv::text")
    public static native void detectRegions(@ByVal opencv_core.UMat uMat, @opencv_core.Ptr ERFilter eRFilter, @opencv_core.Ptr ERFilter eRFilter2, @ByRef opencv_core.PointVectorVector pointVectorVector);

    @Namespace("cv::text")
    public static native void detectRegions(@ByVal opencv_core.UMat uMat, @opencv_core.Ptr ERFilter eRFilter, @opencv_core.Ptr ERFilter eRFilter2, @ByRef opencv_core.RectVector rectVector);

    @Namespace("cv::text")
    public static native void detectRegions(@ByVal opencv_core.UMat uMat, @opencv_core.Ptr ERFilter eRFilter, @opencv_core.Ptr ERFilter eRFilter2, @ByRef opencv_core.RectVector rectVector, int i, @opencv_core.Str String str, float f);

    @Namespace("cv::text")
    public static native void detectRegions(@ByVal opencv_core.UMat uMat, @opencv_core.Ptr ERFilter eRFilter, @opencv_core.Ptr ERFilter eRFilter2, @ByRef opencv_core.RectVector rectVector, int i, @opencv_core.Str BytePointer bytePointer, float f);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector, int i, @opencv_core.Str String str, float f);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @ByVal opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector, int i, @opencv_core.Str BytePointer bytePointer, float f);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByRef ERStatVectorVector eRStatVectorVector, @ByRef @Cast({"std::vector<std::vector<cv::Vec2i> >*"}) opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByRef ERStatVectorVector eRStatVectorVector, @ByRef @Cast({"std::vector<std::vector<cv::Vec2i> >*"}) opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector, int i, @StdString String str, float f);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByRef ERStatVectorVector eRStatVectorVector, @ByRef @Cast({"std::vector<std::vector<cv::Vec2i> >*"}) opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector, int i, @StdString BytePointer bytePointer, float f);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.MatVector matVector, @ByRef ERStatVectorVector eRStatVectorVector, @ByRef @Cast({"std::vector<std::vector<cv::Vec2i> >*"}) opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.MatVector matVector, @ByRef ERStatVectorVector eRStatVectorVector, @ByRef @Cast({"std::vector<std::vector<cv::Vec2i> >*"}) opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector, int i, @StdString String str, float f);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.MatVector matVector, @ByRef ERStatVectorVector eRStatVectorVector, @ByRef @Cast({"std::vector<std::vector<cv::Vec2i> >*"}) opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector, int i, @StdString BytePointer bytePointer, float f);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.UMatVector uMatVector, @ByRef ERStatVectorVector eRStatVectorVector, @ByRef @Cast({"std::vector<std::vector<cv::Vec2i> >*"}) opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.UMatVector uMatVector, @ByRef ERStatVectorVector eRStatVectorVector, @ByRef @Cast({"std::vector<std::vector<cv::Vec2i> >*"}) opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector, int i, @StdString String str, float f);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.UMatVector uMatVector, @ByRef ERStatVectorVector eRStatVectorVector, @ByRef @Cast({"std::vector<std::vector<cv::Vec2i> >*"}) opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector, int i, @StdString BytePointer bytePointer, float f);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.Mat mat, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByRef ERStatVectorVector eRStatVectorVector, @ByRef @Cast({"std::vector<std::vector<cv::Vec2i> >*"}) opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.Mat mat, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByRef ERStatVectorVector eRStatVectorVector, @ByRef @Cast({"std::vector<std::vector<cv::Vec2i> >*"}) opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector, int i, @StdString String str, float f);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.Mat mat, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByRef ERStatVectorVector eRStatVectorVector, @ByRef @Cast({"std::vector<std::vector<cv::Vec2i> >*"}) opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector, int i, @StdString BytePointer bytePointer, float f);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector, int i, @opencv_core.Str String str, float f);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @ByVal opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector, int i, @opencv_core.Str BytePointer bytePointer, float f);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.Mat mat, @ByVal opencv_core.MatVector matVector, @ByRef ERStatVectorVector eRStatVectorVector, @ByRef @Cast({"std::vector<std::vector<cv::Vec2i> >*"}) opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.Mat mat, @ByVal opencv_core.MatVector matVector, @ByRef ERStatVectorVector eRStatVectorVector, @ByRef @Cast({"std::vector<std::vector<cv::Vec2i> >*"}) opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector, int i, @StdString String str, float f);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.Mat mat, @ByVal opencv_core.MatVector matVector, @ByRef ERStatVectorVector eRStatVectorVector, @ByRef @Cast({"std::vector<std::vector<cv::Vec2i> >*"}) opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector, int i, @StdString BytePointer bytePointer, float f);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.Mat mat, @ByVal opencv_core.UMatVector uMatVector, @ByRef ERStatVectorVector eRStatVectorVector, @ByRef @Cast({"std::vector<std::vector<cv::Vec2i> >*"}) opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.Mat mat, @ByVal opencv_core.UMatVector uMatVector, @ByRef ERStatVectorVector eRStatVectorVector, @ByRef @Cast({"std::vector<std::vector<cv::Vec2i> >*"}) opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector, int i, @StdString String str, float f);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.Mat mat, @ByVal opencv_core.UMatVector uMatVector, @ByRef ERStatVectorVector eRStatVectorVector, @ByRef @Cast({"std::vector<std::vector<cv::Vec2i> >*"}) opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector, int i, @StdString BytePointer bytePointer, float f);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByRef ERStatVectorVector eRStatVectorVector, @ByRef @Cast({"std::vector<std::vector<cv::Vec2i> >*"}) opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByRef ERStatVectorVector eRStatVectorVector, @ByRef @Cast({"std::vector<std::vector<cv::Vec2i> >*"}) opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector, int i, @StdString String str, float f);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.GpuMatVector gpuMatVector, @ByRef ERStatVectorVector eRStatVectorVector, @ByRef @Cast({"std::vector<std::vector<cv::Vec2i> >*"}) opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector, int i, @StdString BytePointer bytePointer, float f);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.MatVector matVector, @ByRef ERStatVectorVector eRStatVectorVector, @ByRef @Cast({"std::vector<std::vector<cv::Vec2i> >*"}) opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.MatVector matVector, @ByRef ERStatVectorVector eRStatVectorVector, @ByRef @Cast({"std::vector<std::vector<cv::Vec2i> >*"}) opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector, int i, @StdString String str, float f);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.MatVector matVector, @ByRef ERStatVectorVector eRStatVectorVector, @ByRef @Cast({"std::vector<std::vector<cv::Vec2i> >*"}) opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector, int i, @StdString BytePointer bytePointer, float f);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector, int i, @opencv_core.Str String str, float f);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @ByVal opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector, int i, @opencv_core.Str BytePointer bytePointer, float f);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMatVector uMatVector, @ByRef ERStatVectorVector eRStatVectorVector, @ByRef @Cast({"std::vector<std::vector<cv::Vec2i> >*"}) opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMatVector uMatVector, @ByRef ERStatVectorVector eRStatVectorVector, @ByRef @Cast({"std::vector<std::vector<cv::Vec2i> >*"}) opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector, int i, @StdString String str, float f);

    @Namespace("cv::text")
    public static native void erGrouping(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMatVector uMatVector, @ByRef ERStatVectorVector eRStatVectorVector, @ByRef @Cast({"std::vector<std::vector<cv::Vec2i> >*"}) opencv_core.PointVectorVector pointVectorVector, @ByRef opencv_core.RectVector rectVector, int i, @StdString BytePointer bytePointer, float f);

    @Namespace("cv::text")
    @opencv_core.Ptr
    public static native ERFilter.Callback loadClassifierNM1(@opencv_core.Str String str);

    @Namespace("cv::text")
    @opencv_core.Ptr
    public static native ERFilter.Callback loadClassifierNM1(@opencv_core.Str BytePointer bytePointer);

    @Namespace("cv::text")
    @opencv_core.Ptr
    public static native ERFilter.Callback loadClassifierNM2(@opencv_core.Str String str);

    @Namespace("cv::text")
    @opencv_core.Ptr
    public static native ERFilter.Callback loadClassifierNM2(@opencv_core.Str BytePointer bytePointer);

    @Namespace("cv::text")
    @opencv_core.Ptr
    public static native OCRBeamSearchDecoder.ClassifierCallback loadOCRBeamSearchClassifierCNN(@opencv_core.Str String str);

    @Namespace("cv::text")
    @opencv_core.Ptr
    public static native OCRBeamSearchDecoder.ClassifierCallback loadOCRBeamSearchClassifierCNN(@opencv_core.Str BytePointer bytePointer);

    @Namespace("cv::text")
    @opencv_core.Ptr
    public static native OCRHMMDecoder.ClassifierCallback loadOCRHMMClassifier(@opencv_core.Str String str, int i);

    @Namespace("cv::text")
    @opencv_core.Ptr
    public static native OCRHMMDecoder.ClassifierCallback loadOCRHMMClassifier(@opencv_core.Str BytePointer bytePointer, int i);

    @Namespace("cv::text")
    @opencv_core.Ptr
    public static native OCRHMMDecoder.ClassifierCallback loadOCRHMMClassifierCNN(@opencv_core.Str String str);

    @Namespace("cv::text")
    @opencv_core.Ptr
    public static native OCRHMMDecoder.ClassifierCallback loadOCRHMMClassifierCNN(@opencv_core.Str BytePointer bytePointer);

    @Namespace("cv::text")
    @opencv_core.Ptr
    public static native OCRHMMDecoder.ClassifierCallback loadOCRHMMClassifierNM(@opencv_core.Str String str);

    @Namespace("cv::text")
    @opencv_core.Ptr
    public static native OCRHMMDecoder.ClassifierCallback loadOCRHMMClassifierNM(@opencv_core.Str BytePointer bytePointer);

    static {
        Loader.load();
    }

    @Name({"std::deque<int>"})
    public static class IntDeque extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @Index(function = "at")
        public native int get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, int i);

        public native IntDeque put(@Cast({"size_t"}) long j, int i);

        @ByRef
        @Name({"operator="})
        public native IntDeque put(@ByRef IntDeque intDeque);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public IntDeque(Pointer p) {
            super(p);
        }

        public IntDeque(int... array) {
            this((long) array.length);
            put(array);
        }

        public IntDeque() {
            allocate();
        }

        public IntDeque(long n) {
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

            @Name({"operator*"})
            public native int get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public int[] get() {
            int[] array = new int[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return Arrays.toString(get());
        }

        public int pop_back() {
            long size = size();
            int value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public IntDeque push_back(int value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public IntDeque put(int value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public IntDeque put(int... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<cv::text::ERStat>"})
    public static class ERStatVector extends Pointer {
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
        public native ERStat get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @ByRef ERStat eRStat);

        public native ERStatVector put(@Cast({"size_t"}) long j, ERStat eRStat);

        @ByRef
        @Name({"operator="})
        public native ERStatVector put(@ByRef ERStatVector eRStatVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public ERStatVector(Pointer p) {
            super(p);
        }

        public ERStatVector(ERStat value) {
            this(1);
            put(0, value);
        }

        public ERStatVector(ERStat... array) {
            this((long) array.length);
            put(array);
        }

        public ERStatVector() {
            allocate();
        }

        public ERStatVector(long n) {
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
            public native ERStat get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public ERStat[] get() {
            ERStat[] array = new ERStat[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return Arrays.toString(get());
        }

        public ERStat pop_back() {
            long size = size();
            ERStat value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public ERStatVector push_back(ERStat value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public ERStatVector put(ERStat value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public ERStatVector put(ERStat... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<std::vector<cv::text::ERStat> >"})
    public static class ERStatVectorVector extends Pointer {
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
        public native ERStatVector get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @ByRef ERStatVector eRStatVector);

        public native ERStatVectorVector put(@Cast({"size_t"}) long j, ERStatVector eRStatVector);

        @ByRef
        @Name({"operator="})
        public native ERStatVectorVector put(@ByRef ERStatVectorVector eRStatVectorVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public ERStatVectorVector(Pointer p) {
            super(p);
        }

        public ERStatVectorVector(ERStatVector value) {
            this(1);
            put(0, value);
        }

        public ERStatVectorVector(ERStatVector... array) {
            this((long) array.length);
            put(array);
        }

        public ERStatVectorVector() {
            allocate();
        }

        public ERStatVectorVector(long n) {
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
            public native ERStatVector get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public ERStatVector[] get() {
            ERStatVector[] array = new ERStatVector[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return Arrays.toString(get());
        }

        public ERStatVector pop_back() {
            long size = size();
            ERStatVector value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public ERStatVectorVector push_back(ERStatVector value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public ERStatVectorVector put(ERStatVector value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public ERStatVectorVector put(ERStatVector... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<double>"})
    public static class DoubleVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @Index(function = "at")
        public native double get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, double d);

        public native DoubleVector put(@Cast({"size_t"}) long j, double d);

        @ByRef
        @Name({"operator="})
        public native DoubleVector put(@ByRef DoubleVector doubleVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public DoubleVector(Pointer p) {
            super(p);
        }

        public DoubleVector(double value) {
            this(1);
            put(0, value);
        }

        public DoubleVector(double... array) {
            this((long) array.length);
            put(array);
        }

        public DoubleVector() {
            allocate();
        }

        public DoubleVector(long n) {
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

            @Name({"operator*"})
            public native double get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public double[] get() {
            double[] array = new double[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return Arrays.toString(get());
        }

        public double pop_back() {
            long size = size();
            double value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public DoubleVector push_back(double value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public DoubleVector put(double value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public DoubleVector put(double... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Name({"std::vector<std::string>"})
    public static class StdStringVector extends Pointer {
        private native void allocate();

        private native void allocate(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator begin();

        @ByVal
        public native Iterator end();

        @ByVal
        public native Iterator erase(@ByVal Iterator iterator);

        @Index(function = "at")
        @StdString
        public native BytePointer get(@Cast({"size_t"}) long j);

        @ByVal
        public native Iterator insert(@ByVal Iterator iterator, @StdString BytePointer bytePointer);

        @Index(function = "at")
        @ValueSetter
        public native StdStringVector put(@Cast({"size_t"}) long j, @StdString String str);

        public native StdStringVector put(@Cast({"size_t"}) long j, BytePointer bytePointer);

        @ByRef
        @Name({"operator="})
        public native StdStringVector put(@ByRef StdStringVector stdStringVector);

        public native void resize(@Cast({"size_t"}) long j);

        public native long size();

        static {
            Loader.load();
        }

        public StdStringVector(Pointer p) {
            super(p);
        }

        public StdStringVector(BytePointer value) {
            this(1);
            put(0, value);
        }

        public StdStringVector(BytePointer... array) {
            this((long) array.length);
            put(array);
        }

        public StdStringVector(String value) {
            this(1);
            put(0, value);
        }

        public StdStringVector(String... array) {
            this((long) array.length);
            put(array);
        }

        public StdStringVector() {
            allocate();
        }

        public StdStringVector(long n) {
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

            @StdString
            @Name({"operator*"})
            public native BytePointer get();

            @ByRef
            @Name({"operator++"})
            public native Iterator increment();

            public Iterator(Pointer p) {
                super(p);
            }

            public Iterator() {
            }
        }

        public BytePointer[] get() {
            BytePointer[] array = new BytePointer[(size() < TTL.MAX_VALUE ? (int) size() : Integer.MAX_VALUE)];
            for (int i = 0; i < array.length; i++) {
                array[i] = get((long) i);
            }
            return array;
        }

        public String toString() {
            return Arrays.toString(get());
        }

        public BytePointer pop_back() {
            long size = size();
            BytePointer value = get(size - 1);
            resize(size - 1);
            return value;
        }

        public StdStringVector push_back(BytePointer value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public StdStringVector put(BytePointer value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public StdStringVector put(BytePointer... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }

        public StdStringVector push_back(String value) {
            long size = size();
            resize(1 + size);
            return put(size, value);
        }

        public StdStringVector put(String value) {
            if (size() != 1) {
                resize(1);
            }
            return put(0, value);
        }

        public StdStringVector put(String... array) {
            if (size() != ((long) array.length)) {
                resize((long) array.length);
            }
            for (int i = 0; i < array.length; i++) {
                put((long) i, array[i]);
            }
            return this;
        }
    }

    @Namespace("cv::text")
    @NoOffset
    public static class ERStat extends Pointer {
        private native void allocate();

        private native void allocate(int i, int i2, int i3, int i4);

        private native void allocateArray(long j);

        public native int area();

        public native ERStat area(int i);

        public native double central_moments(int i);

        @MemberGetter
        public native DoublePointer central_moments();

        public native ERStat central_moments(int i, double d);

        public native ERStat child();

        public native ERStat child(ERStat eRStat);

        public native float convex_hull_ratio();

        public native ERStat convex_hull_ratio(float f);

        public native ERStat crossings(IntDeque intDeque);

        @opencv_core.Ptr
        public native IntDeque crossings();

        public native int euler();

        public native ERStat euler(int i);

        public native float hole_area_ratio();

        public native ERStat hole_area_ratio(float f);

        public native int level();

        public native ERStat level(int i);

        public native ERStat local_maxima(boolean z);

        @Cast({"bool"})
        public native boolean local_maxima();

        public native ERStat max_probability_ancestor();

        public native ERStat max_probability_ancestor(ERStat eRStat);

        public native float med_crossings();

        public native ERStat med_crossings(float f);

        public native ERStat min_probability_ancestor();

        public native ERStat min_probability_ancestor(ERStat eRStat);

        public native ERStat next();

        public native ERStat next(ERStat eRStat);

        public native float num_inflexion_points();

        public native ERStat num_inflexion_points(float f);

        public native ERStat parent();

        public native ERStat parent(ERStat eRStat);

        public native int perimeter();

        public native ERStat perimeter(int i);

        public native int pixel();

        public native ERStat pixel(int i);

        @StdVector
        public native IntPointer pixels();

        public native ERStat pixels(IntPointer intPointer);

        public native ERStat prev();

        public native ERStat prev(ERStat eRStat);

        public native double probability();

        public native ERStat probability(double d);

        public native double raw_moments(int i);

        @MemberGetter
        public native DoublePointer raw_moments();

        public native ERStat raw_moments(int i, double d);

        @ByRef
        public native opencv_core.Rect rect();

        public native ERStat rect(opencv_core.Rect rect);

        static {
            Loader.load();
        }

        public ERStat(Pointer p) {
            super(p);
        }

        public ERStat(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public ERStat position(long position) {
            return (ERStat) super.position(position);
        }

        public ERStat(int level, int pixel, int x, int y) {
            super((Pointer) null);
            allocate(level, pixel, x, y);
        }

        public ERStat() {
            super((Pointer) null);
            allocate();
        }
    }

    @Namespace("cv::text")
    public static class ERFilter extends opencv_core.Algorithm {
        public native int getNumRejected();

        public native void run(@ByVal opencv_core.GpuMat gpuMat, @ByRef ERStatVector eRStatVector);

        public native void run(@ByVal opencv_core.Mat mat, @ByRef ERStatVector eRStatVector);

        public native void run(@ByVal opencv_core.UMat uMat, @ByRef ERStatVector eRStatVector);

        public native void setCallback(@opencv_core.Ptr Callback callback);

        public native void setMaxArea(float f);

        public native void setMinArea(float f);

        public native void setMinProbability(float f);

        public native void setMinProbabilityDiff(float f);

        public native void setNonMaxSuppression(@Cast({"bool"}) boolean z);

        public native void setThresholdDelta(int i);

        static {
            Loader.load();
        }

        public ERFilter(Pointer p) {
            super(p);
        }

        public static class Callback extends Pointer {
            public native double eval(@ByRef @Const ERStat eRStat);

            static {
                Loader.load();
            }

            public Callback(Pointer p) {
                super(p);
            }
        }
    }

    @Namespace("cv::text")
    public static class BaseOCR extends Pointer {
        public native void run(@ByRef opencv_core.Mat mat, @ByRef @StdString BytePointer bytePointer);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector FloatBuffer floatBuffer, int i);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector FloatPointer floatPointer, int i);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector float[] fArr, int i);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef opencv_core.Mat mat2, @ByRef @StdString BytePointer bytePointer);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef opencv_core.Mat mat2, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector FloatBuffer floatBuffer, int i);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef opencv_core.Mat mat2, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector FloatPointer floatPointer, int i);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef opencv_core.Mat mat2, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector float[] fArr, int i);

        static {
            Loader.load();
        }

        public BaseOCR(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::text")
    public static class OCRTesseract extends BaseOCR {
        @opencv_core.Ptr
        public static native OCRTesseract create();

        @opencv_core.Ptr
        public static native OCRTesseract create(String str, String str2, String str3, int i, int i2);

        @opencv_core.Ptr
        public static native OCRTesseract create(@Cast({"const char*"}) BytePointer bytePointer, @Cast({"const char*"}) BytePointer bytePointer2, @Cast({"const char*"}) BytePointer bytePointer3, int i, int i2);

        @opencv_core.Str
        public native String run(@ByVal opencv_core.UMat uMat, int i);

        @opencv_core.Str
        public native String run(@ByVal opencv_core.UMat uMat, int i, int i2);

        @opencv_core.Str
        public native String run(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i);

        @opencv_core.Str
        public native String run(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, int i2);

        @opencv_core.Str
        public native BytePointer run(@ByVal opencv_core.GpuMat gpuMat, int i);

        @opencv_core.Str
        public native BytePointer run(@ByVal opencv_core.GpuMat gpuMat, int i, int i2);

        @opencv_core.Str
        public native BytePointer run(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i);

        @opencv_core.Str
        public native BytePointer run(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, int i2);

        @opencv_core.Str
        public native BytePointer run(@ByVal opencv_core.Mat mat, int i);

        @opencv_core.Str
        public native BytePointer run(@ByVal opencv_core.Mat mat, int i, int i2);

        @opencv_core.Str
        public native BytePointer run(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i);

        @opencv_core.Str
        public native BytePointer run(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, int i2);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef @StdString BytePointer bytePointer);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector FloatBuffer floatBuffer, int i);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector FloatPointer floatPointer, int i);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector float[] fArr, int i);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef opencv_core.Mat mat2, @ByRef @StdString BytePointer bytePointer);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef opencv_core.Mat mat2, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector FloatBuffer floatBuffer, int i);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef opencv_core.Mat mat2, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector FloatPointer floatPointer, int i);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef opencv_core.Mat mat2, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector float[] fArr, int i);

        public native void setWhiteList(@opencv_core.Str String str);

        public native void setWhiteList(@opencv_core.Str BytePointer bytePointer);

        static {
            Loader.load();
        }

        public OCRTesseract(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::text")
    @NoOffset
    public static class OCRHMMDecoder extends BaseOCR {
        private native void allocate();

        private native void allocateArray(long j);

        @opencv_core.Ptr
        public static native OCRHMMDecoder create(@opencv_core.Str String str, @opencv_core.Str String str2, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        @opencv_core.Ptr
        public static native OCRHMMDecoder create(@opencv_core.Str String str, @opencv_core.Str String str2, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, int i2);

        @opencv_core.Ptr
        public static native OCRHMMDecoder create(@opencv_core.Str String str, @opencv_core.Str String str2, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        @opencv_core.Ptr
        public static native OCRHMMDecoder create(@opencv_core.Str String str, @opencv_core.Str String str2, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, int i2);

        @opencv_core.Ptr
        public static native OCRHMMDecoder create(@opencv_core.Str String str, @opencv_core.Str String str2, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        @opencv_core.Ptr
        public static native OCRHMMDecoder create(@opencv_core.Str String str, @opencv_core.Str String str2, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, int i2);

        @opencv_core.Ptr
        public static native OCRHMMDecoder create(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        @opencv_core.Ptr
        public static native OCRHMMDecoder create(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, int i2);

        @opencv_core.Ptr
        public static native OCRHMMDecoder create(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        @opencv_core.Ptr
        public static native OCRHMMDecoder create(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, int i2);

        @opencv_core.Ptr
        public static native OCRHMMDecoder create(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        @opencv_core.Ptr
        public static native OCRHMMDecoder create(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, int i2);

        @opencv_core.Ptr
        public static native OCRHMMDecoder create(@opencv_core.Ptr ClassifierCallback classifierCallback, @opencv_core.Str String str, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        @opencv_core.Ptr
        public static native OCRHMMDecoder create(@opencv_core.Ptr ClassifierCallback classifierCallback, @opencv_core.Str String str, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i);

        @opencv_core.Ptr
        public static native OCRHMMDecoder create(@opencv_core.Ptr ClassifierCallback classifierCallback, @opencv_core.Str String str, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        @opencv_core.Ptr
        public static native OCRHMMDecoder create(@opencv_core.Ptr ClassifierCallback classifierCallback, @opencv_core.Str String str, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i);

        @opencv_core.Ptr
        public static native OCRHMMDecoder create(@opencv_core.Ptr ClassifierCallback classifierCallback, @opencv_core.Str String str, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        @opencv_core.Ptr
        public static native OCRHMMDecoder create(@opencv_core.Ptr ClassifierCallback classifierCallback, @opencv_core.Str String str, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i);

        @opencv_core.Ptr
        public static native OCRHMMDecoder create(@opencv_core.Ptr ClassifierCallback classifierCallback, @opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2);

        @opencv_core.Ptr
        public static native OCRHMMDecoder create(@opencv_core.Ptr ClassifierCallback classifierCallback, @opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i);

        @opencv_core.Ptr
        public static native OCRHMMDecoder create(@opencv_core.Ptr ClassifierCallback classifierCallback, @opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2);

        @opencv_core.Ptr
        public static native OCRHMMDecoder create(@opencv_core.Ptr ClassifierCallback classifierCallback, @opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i);

        @opencv_core.Ptr
        public static native OCRHMMDecoder create(@opencv_core.Ptr ClassifierCallback classifierCallback, @opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2);

        @opencv_core.Ptr
        public static native OCRHMMDecoder create(@opencv_core.Ptr ClassifierCallback classifierCallback, @opencv_core.Str BytePointer bytePointer, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i);

        @opencv_core.Str
        public native String run(@ByVal opencv_core.UMat uMat, int i);

        @opencv_core.Str
        public native String run(@ByVal opencv_core.UMat uMat, int i, int i2);

        @opencv_core.Str
        public native String run(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i);

        @opencv_core.Str
        public native String run(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, int i2);

        @opencv_core.Str
        public native BytePointer run(@ByVal opencv_core.GpuMat gpuMat, int i);

        @opencv_core.Str
        public native BytePointer run(@ByVal opencv_core.GpuMat gpuMat, int i, int i2);

        @opencv_core.Str
        public native BytePointer run(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i);

        @opencv_core.Str
        public native BytePointer run(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, int i2);

        @opencv_core.Str
        public native BytePointer run(@ByVal opencv_core.Mat mat, int i);

        @opencv_core.Str
        public native BytePointer run(@ByVal opencv_core.Mat mat, int i, int i2);

        @opencv_core.Str
        public native BytePointer run(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i);

        @opencv_core.Str
        public native BytePointer run(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, int i2);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef @StdString BytePointer bytePointer);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector FloatBuffer floatBuffer, int i);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector FloatPointer floatPointer, int i);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector float[] fArr, int i);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef opencv_core.Mat mat2, @ByRef @StdString BytePointer bytePointer);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef opencv_core.Mat mat2, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector FloatBuffer floatBuffer, int i);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef opencv_core.Mat mat2, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector FloatPointer floatPointer, int i);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef opencv_core.Mat mat2, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector float[] fArr, int i);

        static {
            Loader.load();
        }

        public OCRHMMDecoder() {
            super((Pointer) null);
            allocate();
        }

        public OCRHMMDecoder(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public OCRHMMDecoder(Pointer p) {
            super(p);
        }

        public OCRHMMDecoder position(long position) {
            return (OCRHMMDecoder) super.position(position);
        }

        public static class ClassifierCallback extends Pointer {
            private native void allocate();

            private native void allocateArray(long j);

            public native void eval(@ByVal opencv_core.GpuMat gpuMat, @StdVector IntBuffer intBuffer, @ByRef DoubleVector doubleVector);

            public native void eval(@ByVal opencv_core.GpuMat gpuMat, @StdVector IntPointer intPointer, @ByRef DoubleVector doubleVector);

            public native void eval(@ByVal opencv_core.GpuMat gpuMat, @StdVector int[] iArr, @ByRef DoubleVector doubleVector);

            public native void eval(@ByVal opencv_core.Mat mat, @StdVector IntBuffer intBuffer, @ByRef DoubleVector doubleVector);

            public native void eval(@ByVal opencv_core.Mat mat, @StdVector IntPointer intPointer, @ByRef DoubleVector doubleVector);

            public native void eval(@ByVal opencv_core.Mat mat, @StdVector int[] iArr, @ByRef DoubleVector doubleVector);

            public native void eval(@ByVal opencv_core.UMat uMat, @StdVector IntBuffer intBuffer, @ByRef DoubleVector doubleVector);

            public native void eval(@ByVal opencv_core.UMat uMat, @StdVector IntPointer intPointer, @ByRef DoubleVector doubleVector);

            public native void eval(@ByVal opencv_core.UMat uMat, @StdVector int[] iArr, @ByRef DoubleVector doubleVector);

            static {
                Loader.load();
            }

            public ClassifierCallback() {
                super((Pointer) null);
                allocate();
            }

            public ClassifierCallback(long size) {
                super((Pointer) null);
                allocateArray(size);
            }

            public ClassifierCallback(Pointer p) {
                super(p);
            }

            public ClassifierCallback position(long position) {
                return (ClassifierCallback) super.position(position);
            }
        }
    }

    @Namespace("cv::text")
    @NoOffset
    public static class OCRBeamSearchDecoder extends BaseOCR {
        private native void allocate();

        private native void allocateArray(long j);

        @opencv_core.Ptr
        public static native OCRBeamSearchDecoder create(@opencv_core.Str String str, @opencv_core.Str String str2, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, int i2);

        @opencv_core.Ptr
        public static native OCRBeamSearchDecoder create(@opencv_core.Str String str, @opencv_core.Str String str2, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, int i2);

        @opencv_core.Ptr
        public static native OCRBeamSearchDecoder create(@opencv_core.Str String str, @opencv_core.Str String str2, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, int i2);

        @opencv_core.Ptr
        public static native OCRBeamSearchDecoder create(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, int i2);

        @opencv_core.Ptr
        public static native OCRBeamSearchDecoder create(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, int i2);

        @opencv_core.Ptr
        public static native OCRBeamSearchDecoder create(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, int i2);

        @opencv_core.Ptr
        public static native OCRBeamSearchDecoder create(@opencv_core.Ptr ClassifierCallback classifierCallback, @StdString String str, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @Cast({"cv::text::decoder_mode"}) int i, int i2);

        @opencv_core.Ptr
        public static native OCRBeamSearchDecoder create(@opencv_core.Ptr ClassifierCallback classifierCallback, @StdString String str, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @Cast({"cv::text::decoder_mode"}) int i, int i2);

        @opencv_core.Ptr
        public static native OCRBeamSearchDecoder create(@opencv_core.Ptr ClassifierCallback classifierCallback, @StdString String str, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @Cast({"cv::text::decoder_mode"}) int i, int i2);

        @opencv_core.Ptr
        public static native OCRBeamSearchDecoder create(@opencv_core.Ptr ClassifierCallback classifierCallback, @StdString BytePointer bytePointer, @ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, @Cast({"cv::text::decoder_mode"}) int i, int i2);

        @opencv_core.Ptr
        public static native OCRBeamSearchDecoder create(@opencv_core.Ptr ClassifierCallback classifierCallback, @StdString BytePointer bytePointer, @ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, @Cast({"cv::text::decoder_mode"}) int i, int i2);

        @opencv_core.Ptr
        public static native OCRBeamSearchDecoder create(@opencv_core.Ptr ClassifierCallback classifierCallback, @StdString BytePointer bytePointer, @ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, @Cast({"cv::text::decoder_mode"}) int i, int i2);

        @opencv_core.Str
        public native String run(@ByVal opencv_core.UMat uMat, int i);

        @opencv_core.Str
        public native String run(@ByVal opencv_core.UMat uMat, int i, int i2);

        @opencv_core.Str
        public native String run(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i);

        @opencv_core.Str
        public native String run(@ByVal opencv_core.UMat uMat, @ByVal opencv_core.UMat uMat2, int i, int i2);

        @opencv_core.Str
        public native BytePointer run(@ByVal opencv_core.GpuMat gpuMat, int i);

        @opencv_core.Str
        public native BytePointer run(@ByVal opencv_core.GpuMat gpuMat, int i, int i2);

        @opencv_core.Str
        public native BytePointer run(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i);

        @opencv_core.Str
        public native BytePointer run(@ByVal opencv_core.GpuMat gpuMat, @ByVal opencv_core.GpuMat gpuMat2, int i, int i2);

        @opencv_core.Str
        public native BytePointer run(@ByVal opencv_core.Mat mat, int i);

        @opencv_core.Str
        public native BytePointer run(@ByVal opencv_core.Mat mat, int i, int i2);

        @opencv_core.Str
        public native BytePointer run(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i);

        @opencv_core.Str
        public native BytePointer run(@ByVal opencv_core.Mat mat, @ByVal opencv_core.Mat mat2, int i, int i2);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef @StdString BytePointer bytePointer);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector FloatBuffer floatBuffer, int i);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector FloatPointer floatPointer, int i);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector float[] fArr, int i);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef opencv_core.Mat mat2, @ByRef @StdString BytePointer bytePointer);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef opencv_core.Mat mat2, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector FloatBuffer floatBuffer, int i);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef opencv_core.Mat mat2, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector FloatPointer floatPointer, int i);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef opencv_core.Mat mat2, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector float[] fArr, int i);

        static {
            Loader.load();
        }

        public OCRBeamSearchDecoder() {
            super((Pointer) null);
            allocate();
        }

        public OCRBeamSearchDecoder(long size) {
            super((Pointer) null);
            allocateArray(size);
        }

        public OCRBeamSearchDecoder(Pointer p) {
            super(p);
        }

        public OCRBeamSearchDecoder position(long position) {
            return (OCRBeamSearchDecoder) super.position(position);
        }

        public static class ClassifierCallback extends Pointer {
            private native void allocate();

            private native void allocateArray(long j);

            public native void eval(@ByVal opencv_core.GpuMat gpuMat, @StdVector DoubleVector doubleVector, @StdVector IntBuffer intBuffer);

            public native void eval(@ByVal opencv_core.GpuMat gpuMat, @StdVector DoubleVector doubleVector, @StdVector IntPointer intPointer);

            public native void eval(@ByVal opencv_core.GpuMat gpuMat, @StdVector DoubleVector doubleVector, @StdVector int[] iArr);

            public native void eval(@ByVal opencv_core.Mat mat, @StdVector DoubleVector doubleVector, @StdVector IntBuffer intBuffer);

            public native void eval(@ByVal opencv_core.Mat mat, @StdVector DoubleVector doubleVector, @StdVector IntPointer intPointer);

            public native void eval(@ByVal opencv_core.Mat mat, @StdVector DoubleVector doubleVector, @StdVector int[] iArr);

            public native void eval(@ByVal opencv_core.UMat uMat, @StdVector DoubleVector doubleVector, @StdVector IntBuffer intBuffer);

            public native void eval(@ByVal opencv_core.UMat uMat, @StdVector DoubleVector doubleVector, @StdVector IntPointer intPointer);

            public native void eval(@ByVal opencv_core.UMat uMat, @StdVector DoubleVector doubleVector, @StdVector int[] iArr);

            public native int getStepSize();

            public native int getWindowSize();

            static {
                Loader.load();
            }

            public ClassifierCallback() {
                super((Pointer) null);
                allocate();
            }

            public ClassifierCallback(long size) {
                super((Pointer) null);
                allocateArray(size);
            }

            public ClassifierCallback(Pointer p) {
                super(p);
            }

            public ClassifierCallback position(long position) {
                return (ClassifierCallback) super.position(position);
            }
        }
    }

    @Namespace("cv::text")
    public static class OCRHolisticWordRecognizer extends BaseOCR {
        @opencv_core.Ptr
        public static native OCRHolisticWordRecognizer create(@StdString String str, @StdString String str2, @StdString String str3);

        @opencv_core.Ptr
        public static native OCRHolisticWordRecognizer create(@StdString BytePointer bytePointer, @StdString BytePointer bytePointer2, @StdString BytePointer bytePointer3);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef @StdString BytePointer bytePointer);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector FloatBuffer floatBuffer, int i);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector FloatPointer floatPointer, int i);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector float[] fArr, int i);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef opencv_core.Mat mat2, @ByRef @StdString BytePointer bytePointer);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef opencv_core.Mat mat2, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector FloatBuffer floatBuffer, int i);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef opencv_core.Mat mat2, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector FloatPointer floatPointer, int i);

        public native void run(@ByRef opencv_core.Mat mat, @ByRef opencv_core.Mat mat2, @ByRef @StdString BytePointer bytePointer, opencv_core.RectVector rectVector, StdStringVector stdStringVector, @StdVector float[] fArr, int i);

        static {
            Loader.load();
        }

        public OCRHolisticWordRecognizer(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::text")
    public static class TextDetector extends Pointer {
        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector FloatBuffer floatBuffer);

        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector FloatPointer floatPointer);

        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector float[] fArr);

        public native void detect(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector FloatBuffer floatBuffer);

        public native void detect(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector FloatPointer floatPointer);

        public native void detect(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector float[] fArr);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector FloatBuffer floatBuffer);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector FloatPointer floatPointer);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector float[] fArr);

        static {
            Loader.load();
        }

        public TextDetector(Pointer p) {
            super(p);
        }
    }

    @Namespace("cv::text")
    public static class TextDetectorCNN extends TextDetector {
        @opencv_core.Ptr
        public static native TextDetectorCNN create(@opencv_core.Str String str, @opencv_core.Str String str2);

        @opencv_core.Ptr
        public static native TextDetectorCNN create(@opencv_core.Str String str, @opencv_core.Str String str2, @ByVal opencv_core.SizeVector sizeVector);

        @opencv_core.Ptr
        public static native TextDetectorCNN create(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2);

        @opencv_core.Ptr
        public static native TextDetectorCNN create(@opencv_core.Str BytePointer bytePointer, @opencv_core.Str BytePointer bytePointer2, @ByVal opencv_core.SizeVector sizeVector);

        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector FloatBuffer floatBuffer);

        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector FloatPointer floatPointer);

        public native void detect(@ByVal opencv_core.GpuMat gpuMat, @ByRef opencv_core.RectVector rectVector, @StdVector float[] fArr);

        public native void detect(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector FloatBuffer floatBuffer);

        public native void detect(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector FloatPointer floatPointer);

        public native void detect(@ByVal opencv_core.Mat mat, @ByRef opencv_core.RectVector rectVector, @StdVector float[] fArr);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector FloatBuffer floatBuffer);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector FloatPointer floatPointer);

        public native void detect(@ByVal opencv_core.UMat uMat, @ByRef opencv_core.RectVector rectVector, @StdVector float[] fArr);

        static {
            Loader.load();
        }

        public TextDetectorCNN(Pointer p) {
            super(p);
        }
    }
}
