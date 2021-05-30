package org.opencv.photo;

import java.util.List;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.Point;
import org.opencv.utils.Converters;

public class Photo {
    public static final int INPAINT_NS = 0;
    public static final int INPAINT_TELEA = 1;
    public static final int LDR_SIZE = 256;
    public static final int MIXED_CLONE = 2;
    public static final int MONOCHROME_TRANSFER = 3;
    public static final int NORMAL_CLONE = 1;
    public static final int NORMCONV_FILTER = 2;
    public static final int RECURS_FILTER = 1;

    private static native void colorChange_0(long j, long j2, long j3, float f, float f2, float f3);

    private static native void colorChange_1(long j, long j2, long j3, float f, float f2);

    private static native void colorChange_2(long j, long j2, long j3, float f);

    private static native void colorChange_3(long j, long j2, long j3);

    private static native long createAlignMTB_0(int i, int i2, boolean z);

    private static native long createAlignMTB_1(int i, int i2);

    private static native long createAlignMTB_2(int i);

    private static native long createAlignMTB_3();

    private static native long createCalibrateDebevec_0(int i, float f, boolean z);

    private static native long createCalibrateDebevec_1(int i, float f);

    private static native long createCalibrateDebevec_2(int i);

    private static native long createCalibrateDebevec_3();

    private static native long createCalibrateRobertson_0(int i, float f);

    private static native long createCalibrateRobertson_1(int i);

    private static native long createCalibrateRobertson_2();

    private static native long createMergeDebevec_0();

    private static native long createMergeMertens_0(float f, float f2, float f3);

    private static native long createMergeMertens_1(float f, float f2);

    private static native long createMergeMertens_2(float f);

    private static native long createMergeMertens_3();

    private static native long createMergeRobertson_0();

    private static native long createTonemapDrago_0(float f, float f2, float f3);

    private static native long createTonemapDrago_1(float f, float f2);

    private static native long createTonemapDrago_2(float f);

    private static native long createTonemapDrago_3();

    private static native long createTonemapMantiuk_0(float f, float f2, float f3);

    private static native long createTonemapMantiuk_1(float f, float f2);

    private static native long createTonemapMantiuk_2(float f);

    private static native long createTonemapMantiuk_3();

    private static native long createTonemapReinhard_0(float f, float f2, float f3, float f4);

    private static native long createTonemapReinhard_1(float f, float f2, float f3);

    private static native long createTonemapReinhard_2(float f, float f2);

    private static native long createTonemapReinhard_3(float f);

    private static native long createTonemapReinhard_4();

    private static native long createTonemap_0(float f);

    private static native long createTonemap_1();

    private static native void decolor_0(long j, long j2, long j3);

    private static native void denoise_TVL1_0(long j, long j2, double d, int i);

    private static native void denoise_TVL1_1(long j, long j2, double d);

    private static native void denoise_TVL1_2(long j, long j2);

    private static native void detailEnhance_0(long j, long j2, float f, float f2);

    private static native void detailEnhance_1(long j, long j2, float f);

    private static native void detailEnhance_2(long j, long j2);

    private static native void edgePreservingFilter_0(long j, long j2, int i, float f, float f2);

    private static native void edgePreservingFilter_1(long j, long j2, int i, float f);

    private static native void edgePreservingFilter_2(long j, long j2, int i);

    private static native void edgePreservingFilter_3(long j, long j2);

    private static native void fastNlMeansDenoisingColoredMulti_0(long j, long j2, int i, int i2, float f, float f2, int i3, int i4);

    private static native void fastNlMeansDenoisingColoredMulti_1(long j, long j2, int i, int i2, float f, float f2, int i3);

    private static native void fastNlMeansDenoisingColoredMulti_2(long j, long j2, int i, int i2, float f, float f2);

    private static native void fastNlMeansDenoisingColoredMulti_3(long j, long j2, int i, int i2, float f);

    private static native void fastNlMeansDenoisingColoredMulti_4(long j, long j2, int i, int i2);

    private static native void fastNlMeansDenoisingColored_0(long j, long j2, float f, float f2, int i, int i2);

    private static native void fastNlMeansDenoisingColored_1(long j, long j2, float f, float f2, int i);

    private static native void fastNlMeansDenoisingColored_2(long j, long j2, float f, float f2);

    private static native void fastNlMeansDenoisingColored_3(long j, long j2, float f);

    private static native void fastNlMeansDenoisingColored_4(long j, long j2);

    private static native void fastNlMeansDenoisingMulti_0(long j, long j2, int i, int i2, float f, int i3, int i4);

    private static native void fastNlMeansDenoisingMulti_1(long j, long j2, int i, int i2, float f, int i3);

    private static native void fastNlMeansDenoisingMulti_2(long j, long j2, int i, int i2, float f);

    private static native void fastNlMeansDenoisingMulti_3(long j, long j2, int i, int i2);

    private static native void fastNlMeansDenoisingMulti_4(long j, long j2, int i, int i2, long j3, int i3, int i4, int i5);

    private static native void fastNlMeansDenoisingMulti_5(long j, long j2, int i, int i2, long j3, int i3, int i4);

    private static native void fastNlMeansDenoisingMulti_6(long j, long j2, int i, int i2, long j3, int i3);

    private static native void fastNlMeansDenoisingMulti_7(long j, long j2, int i, int i2, long j3);

    private static native void fastNlMeansDenoising_0(long j, long j2, float f, int i, int i2);

    private static native void fastNlMeansDenoising_1(long j, long j2, float f, int i);

    private static native void fastNlMeansDenoising_2(long j, long j2, float f);

    private static native void fastNlMeansDenoising_3(long j, long j2);

    private static native void fastNlMeansDenoising_4(long j, long j2, long j3, int i, int i2, int i3);

    private static native void fastNlMeansDenoising_5(long j, long j2, long j3, int i, int i2);

    private static native void fastNlMeansDenoising_6(long j, long j2, long j3, int i);

    private static native void fastNlMeansDenoising_7(long j, long j2, long j3);

    private static native void illuminationChange_0(long j, long j2, long j3, float f, float f2);

    private static native void illuminationChange_1(long j, long j2, long j3, float f);

    private static native void illuminationChange_2(long j, long j2, long j3);

    private static native void inpaint_0(long j, long j2, long j3, double d, int i);

    private static native void pencilSketch_0(long j, long j2, long j3, float f, float f2, float f3);

    private static native void pencilSketch_1(long j, long j2, long j3, float f, float f2);

    private static native void pencilSketch_2(long j, long j2, long j3, float f);

    private static native void pencilSketch_3(long j, long j2, long j3);

    private static native void seamlessClone_0(long j, long j2, long j3, double d, double d2, long j4, int i);

    private static native void stylization_0(long j, long j2, float f, float f2);

    private static native void stylization_1(long j, long j2, float f);

    private static native void stylization_2(long j, long j2);

    private static native void textureFlattening_0(long j, long j2, long j3, float f, float f2, int i);

    private static native void textureFlattening_1(long j, long j2, long j3, float f, float f2);

    private static native void textureFlattening_2(long j, long j2, long j3, float f);

    private static native void textureFlattening_3(long j, long j2, long j3);

    public static AlignMTB createAlignMTB(int max_bits, int exclude_range, boolean cut) {
        return AlignMTB.__fromPtr__(createAlignMTB_0(max_bits, exclude_range, cut));
    }

    public static AlignMTB createAlignMTB(int max_bits, int exclude_range) {
        return AlignMTB.__fromPtr__(createAlignMTB_1(max_bits, exclude_range));
    }

    public static AlignMTB createAlignMTB(int max_bits) {
        return AlignMTB.__fromPtr__(createAlignMTB_2(max_bits));
    }

    public static AlignMTB createAlignMTB() {
        return AlignMTB.__fromPtr__(createAlignMTB_3());
    }

    public static CalibrateDebevec createCalibrateDebevec(int samples, float lambda, boolean random) {
        return CalibrateDebevec.__fromPtr__(createCalibrateDebevec_0(samples, lambda, random));
    }

    public static CalibrateDebevec createCalibrateDebevec(int samples, float lambda) {
        return CalibrateDebevec.__fromPtr__(createCalibrateDebevec_1(samples, lambda));
    }

    public static CalibrateDebevec createCalibrateDebevec(int samples) {
        return CalibrateDebevec.__fromPtr__(createCalibrateDebevec_2(samples));
    }

    public static CalibrateDebevec createCalibrateDebevec() {
        return CalibrateDebevec.__fromPtr__(createCalibrateDebevec_3());
    }

    public static CalibrateRobertson createCalibrateRobertson(int max_iter, float threshold) {
        return CalibrateRobertson.__fromPtr__(createCalibrateRobertson_0(max_iter, threshold));
    }

    public static CalibrateRobertson createCalibrateRobertson(int max_iter) {
        return CalibrateRobertson.__fromPtr__(createCalibrateRobertson_1(max_iter));
    }

    public static CalibrateRobertson createCalibrateRobertson() {
        return CalibrateRobertson.__fromPtr__(createCalibrateRobertson_2());
    }

    public static MergeDebevec createMergeDebevec() {
        return MergeDebevec.__fromPtr__(createMergeDebevec_0());
    }

    public static MergeMertens createMergeMertens(float contrast_weight, float saturation_weight, float exposure_weight) {
        return MergeMertens.__fromPtr__(createMergeMertens_0(contrast_weight, saturation_weight, exposure_weight));
    }

    public static MergeMertens createMergeMertens(float contrast_weight, float saturation_weight) {
        return MergeMertens.__fromPtr__(createMergeMertens_1(contrast_weight, saturation_weight));
    }

    public static MergeMertens createMergeMertens(float contrast_weight) {
        return MergeMertens.__fromPtr__(createMergeMertens_2(contrast_weight));
    }

    public static MergeMertens createMergeMertens() {
        return MergeMertens.__fromPtr__(createMergeMertens_3());
    }

    public static MergeRobertson createMergeRobertson() {
        return MergeRobertson.__fromPtr__(createMergeRobertson_0());
    }

    public static Tonemap createTonemap(float gamma) {
        return Tonemap.__fromPtr__(createTonemap_0(gamma));
    }

    public static Tonemap createTonemap() {
        return Tonemap.__fromPtr__(createTonemap_1());
    }

    public static TonemapDrago createTonemapDrago(float gamma, float saturation, float bias) {
        return TonemapDrago.__fromPtr__(createTonemapDrago_0(gamma, saturation, bias));
    }

    public static TonemapDrago createTonemapDrago(float gamma, float saturation) {
        return TonemapDrago.__fromPtr__(createTonemapDrago_1(gamma, saturation));
    }

    public static TonemapDrago createTonemapDrago(float gamma) {
        return TonemapDrago.__fromPtr__(createTonemapDrago_2(gamma));
    }

    public static TonemapDrago createTonemapDrago() {
        return TonemapDrago.__fromPtr__(createTonemapDrago_3());
    }

    public static TonemapMantiuk createTonemapMantiuk(float gamma, float scale, float saturation) {
        return TonemapMantiuk.__fromPtr__(createTonemapMantiuk_0(gamma, scale, saturation));
    }

    public static TonemapMantiuk createTonemapMantiuk(float gamma, float scale) {
        return TonemapMantiuk.__fromPtr__(createTonemapMantiuk_1(gamma, scale));
    }

    public static TonemapMantiuk createTonemapMantiuk(float gamma) {
        return TonemapMantiuk.__fromPtr__(createTonemapMantiuk_2(gamma));
    }

    public static TonemapMantiuk createTonemapMantiuk() {
        return TonemapMantiuk.__fromPtr__(createTonemapMantiuk_3());
    }

    public static TonemapReinhard createTonemapReinhard(float gamma, float intensity, float light_adapt, float color_adapt) {
        return TonemapReinhard.__fromPtr__(createTonemapReinhard_0(gamma, intensity, light_adapt, color_adapt));
    }

    public static TonemapReinhard createTonemapReinhard(float gamma, float intensity, float light_adapt) {
        return TonemapReinhard.__fromPtr__(createTonemapReinhard_1(gamma, intensity, light_adapt));
    }

    public static TonemapReinhard createTonemapReinhard(float gamma, float intensity) {
        return TonemapReinhard.__fromPtr__(createTonemapReinhard_2(gamma, intensity));
    }

    public static TonemapReinhard createTonemapReinhard(float gamma) {
        return TonemapReinhard.__fromPtr__(createTonemapReinhard_3(gamma));
    }

    public static TonemapReinhard createTonemapReinhard() {
        return TonemapReinhard.__fromPtr__(createTonemapReinhard_4());
    }

    public static void colorChange(Mat src, Mat mask, Mat dst, float red_mul, float green_mul, float blue_mul) {
        colorChange_0(src.nativeObj, mask.nativeObj, dst.nativeObj, red_mul, green_mul, blue_mul);
    }

    public static void colorChange(Mat src, Mat mask, Mat dst, float red_mul, float green_mul) {
        colorChange_1(src.nativeObj, mask.nativeObj, dst.nativeObj, red_mul, green_mul);
    }

    public static void colorChange(Mat src, Mat mask, Mat dst, float red_mul) {
        colorChange_2(src.nativeObj, mask.nativeObj, dst.nativeObj, red_mul);
    }

    public static void colorChange(Mat src, Mat mask, Mat dst) {
        colorChange_3(src.nativeObj, mask.nativeObj, dst.nativeObj);
    }

    public static void decolor(Mat src, Mat grayscale, Mat color_boost) {
        decolor_0(src.nativeObj, grayscale.nativeObj, color_boost.nativeObj);
    }

    public static void denoise_TVL1(List<Mat> observations, Mat result, double lambda, int niters) {
        denoise_TVL1_0(Converters.vector_Mat_to_Mat(observations).nativeObj, result.nativeObj, lambda, niters);
    }

    public static void denoise_TVL1(List<Mat> observations, Mat result, double lambda) {
        denoise_TVL1_1(Converters.vector_Mat_to_Mat(observations).nativeObj, result.nativeObj, lambda);
    }

    public static void denoise_TVL1(List<Mat> observations, Mat result) {
        denoise_TVL1_2(Converters.vector_Mat_to_Mat(observations).nativeObj, result.nativeObj);
    }

    public static void detailEnhance(Mat src, Mat dst, float sigma_s, float sigma_r) {
        detailEnhance_0(src.nativeObj, dst.nativeObj, sigma_s, sigma_r);
    }

    public static void detailEnhance(Mat src, Mat dst, float sigma_s) {
        detailEnhance_1(src.nativeObj, dst.nativeObj, sigma_s);
    }

    public static void detailEnhance(Mat src, Mat dst) {
        detailEnhance_2(src.nativeObj, dst.nativeObj);
    }

    public static void edgePreservingFilter(Mat src, Mat dst, int flags, float sigma_s, float sigma_r) {
        edgePreservingFilter_0(src.nativeObj, dst.nativeObj, flags, sigma_s, sigma_r);
    }

    public static void edgePreservingFilter(Mat src, Mat dst, int flags, float sigma_s) {
        edgePreservingFilter_1(src.nativeObj, dst.nativeObj, flags, sigma_s);
    }

    public static void edgePreservingFilter(Mat src, Mat dst, int flags) {
        edgePreservingFilter_2(src.nativeObj, dst.nativeObj, flags);
    }

    public static void edgePreservingFilter(Mat src, Mat dst) {
        edgePreservingFilter_3(src.nativeObj, dst.nativeObj);
    }

    public static void fastNlMeansDenoising(Mat src, Mat dst, float h, int templateWindowSize, int searchWindowSize) {
        fastNlMeansDenoising_0(src.nativeObj, dst.nativeObj, h, templateWindowSize, searchWindowSize);
    }

    public static void fastNlMeansDenoising(Mat src, Mat dst, float h, int templateWindowSize) {
        fastNlMeansDenoising_1(src.nativeObj, dst.nativeObj, h, templateWindowSize);
    }

    public static void fastNlMeansDenoising(Mat src, Mat dst, float h) {
        fastNlMeansDenoising_2(src.nativeObj, dst.nativeObj, h);
    }

    public static void fastNlMeansDenoising(Mat src, Mat dst) {
        fastNlMeansDenoising_3(src.nativeObj, dst.nativeObj);
    }

    public static void fastNlMeansDenoising(Mat src, Mat dst, MatOfFloat h, int templateWindowSize, int searchWindowSize, int normType) {
        fastNlMeansDenoising_4(src.nativeObj, dst.nativeObj, h.nativeObj, templateWindowSize, searchWindowSize, normType);
    }

    public static void fastNlMeansDenoising(Mat src, Mat dst, MatOfFloat h, int templateWindowSize, int searchWindowSize) {
        fastNlMeansDenoising_5(src.nativeObj, dst.nativeObj, h.nativeObj, templateWindowSize, searchWindowSize);
    }

    public static void fastNlMeansDenoising(Mat src, Mat dst, MatOfFloat h, int templateWindowSize) {
        fastNlMeansDenoising_6(src.nativeObj, dst.nativeObj, h.nativeObj, templateWindowSize);
    }

    public static void fastNlMeansDenoising(Mat src, Mat dst, MatOfFloat h) {
        fastNlMeansDenoising_7(src.nativeObj, dst.nativeObj, h.nativeObj);
    }

    public static void fastNlMeansDenoisingColored(Mat src, Mat dst, float h, float hColor, int templateWindowSize, int searchWindowSize) {
        fastNlMeansDenoisingColored_0(src.nativeObj, dst.nativeObj, h, hColor, templateWindowSize, searchWindowSize);
    }

    public static void fastNlMeansDenoisingColored(Mat src, Mat dst, float h, float hColor, int templateWindowSize) {
        fastNlMeansDenoisingColored_1(src.nativeObj, dst.nativeObj, h, hColor, templateWindowSize);
    }

    public static void fastNlMeansDenoisingColored(Mat src, Mat dst, float h, float hColor) {
        fastNlMeansDenoisingColored_2(src.nativeObj, dst.nativeObj, h, hColor);
    }

    public static void fastNlMeansDenoisingColored(Mat src, Mat dst, float h) {
        fastNlMeansDenoisingColored_3(src.nativeObj, dst.nativeObj, h);
    }

    public static void fastNlMeansDenoisingColored(Mat src, Mat dst) {
        fastNlMeansDenoisingColored_4(src.nativeObj, dst.nativeObj);
    }

    public static void fastNlMeansDenoisingColoredMulti(List<Mat> srcImgs, Mat dst, int imgToDenoiseIndex, int temporalWindowSize, float h, float hColor, int templateWindowSize, int searchWindowSize) {
        fastNlMeansDenoisingColoredMulti_0(Converters.vector_Mat_to_Mat(srcImgs).nativeObj, dst.nativeObj, imgToDenoiseIndex, temporalWindowSize, h, hColor, templateWindowSize, searchWindowSize);
    }

    public static void fastNlMeansDenoisingColoredMulti(List<Mat> srcImgs, Mat dst, int imgToDenoiseIndex, int temporalWindowSize, float h, float hColor, int templateWindowSize) {
        fastNlMeansDenoisingColoredMulti_1(Converters.vector_Mat_to_Mat(srcImgs).nativeObj, dst.nativeObj, imgToDenoiseIndex, temporalWindowSize, h, hColor, templateWindowSize);
    }

    public static void fastNlMeansDenoisingColoredMulti(List<Mat> srcImgs, Mat dst, int imgToDenoiseIndex, int temporalWindowSize, float h, float hColor) {
        fastNlMeansDenoisingColoredMulti_2(Converters.vector_Mat_to_Mat(srcImgs).nativeObj, dst.nativeObj, imgToDenoiseIndex, temporalWindowSize, h, hColor);
    }

    public static void fastNlMeansDenoisingColoredMulti(List<Mat> srcImgs, Mat dst, int imgToDenoiseIndex, int temporalWindowSize, float h) {
        fastNlMeansDenoisingColoredMulti_3(Converters.vector_Mat_to_Mat(srcImgs).nativeObj, dst.nativeObj, imgToDenoiseIndex, temporalWindowSize, h);
    }

    public static void fastNlMeansDenoisingColoredMulti(List<Mat> srcImgs, Mat dst, int imgToDenoiseIndex, int temporalWindowSize) {
        fastNlMeansDenoisingColoredMulti_4(Converters.vector_Mat_to_Mat(srcImgs).nativeObj, dst.nativeObj, imgToDenoiseIndex, temporalWindowSize);
    }

    public static void fastNlMeansDenoisingMulti(List<Mat> srcImgs, Mat dst, int imgToDenoiseIndex, int temporalWindowSize, float h, int templateWindowSize, int searchWindowSize) {
        fastNlMeansDenoisingMulti_0(Converters.vector_Mat_to_Mat(srcImgs).nativeObj, dst.nativeObj, imgToDenoiseIndex, temporalWindowSize, h, templateWindowSize, searchWindowSize);
    }

    public static void fastNlMeansDenoisingMulti(List<Mat> srcImgs, Mat dst, int imgToDenoiseIndex, int temporalWindowSize, float h, int templateWindowSize) {
        fastNlMeansDenoisingMulti_1(Converters.vector_Mat_to_Mat(srcImgs).nativeObj, dst.nativeObj, imgToDenoiseIndex, temporalWindowSize, h, templateWindowSize);
    }

    public static void fastNlMeansDenoisingMulti(List<Mat> srcImgs, Mat dst, int imgToDenoiseIndex, int temporalWindowSize, float h) {
        fastNlMeansDenoisingMulti_2(Converters.vector_Mat_to_Mat(srcImgs).nativeObj, dst.nativeObj, imgToDenoiseIndex, temporalWindowSize, h);
    }

    public static void fastNlMeansDenoisingMulti(List<Mat> srcImgs, Mat dst, int imgToDenoiseIndex, int temporalWindowSize) {
        fastNlMeansDenoisingMulti_3(Converters.vector_Mat_to_Mat(srcImgs).nativeObj, dst.nativeObj, imgToDenoiseIndex, temporalWindowSize);
    }

    public static void fastNlMeansDenoisingMulti(List<Mat> srcImgs, Mat dst, int imgToDenoiseIndex, int temporalWindowSize, MatOfFloat h, int templateWindowSize, int searchWindowSize, int normType) {
        fastNlMeansDenoisingMulti_4(Converters.vector_Mat_to_Mat(srcImgs).nativeObj, dst.nativeObj, imgToDenoiseIndex, temporalWindowSize, h.nativeObj, templateWindowSize, searchWindowSize, normType);
    }

    public static void fastNlMeansDenoisingMulti(List<Mat> srcImgs, Mat dst, int imgToDenoiseIndex, int temporalWindowSize, MatOfFloat h, int templateWindowSize, int searchWindowSize) {
        fastNlMeansDenoisingMulti_5(Converters.vector_Mat_to_Mat(srcImgs).nativeObj, dst.nativeObj, imgToDenoiseIndex, temporalWindowSize, h.nativeObj, templateWindowSize, searchWindowSize);
    }

    public static void fastNlMeansDenoisingMulti(List<Mat> srcImgs, Mat dst, int imgToDenoiseIndex, int temporalWindowSize, MatOfFloat h, int templateWindowSize) {
        fastNlMeansDenoisingMulti_6(Converters.vector_Mat_to_Mat(srcImgs).nativeObj, dst.nativeObj, imgToDenoiseIndex, temporalWindowSize, h.nativeObj, templateWindowSize);
    }

    public static void fastNlMeansDenoisingMulti(List<Mat> srcImgs, Mat dst, int imgToDenoiseIndex, int temporalWindowSize, MatOfFloat h) {
        fastNlMeansDenoisingMulti_7(Converters.vector_Mat_to_Mat(srcImgs).nativeObj, dst.nativeObj, imgToDenoiseIndex, temporalWindowSize, h.nativeObj);
    }

    public static void illuminationChange(Mat src, Mat mask, Mat dst, float alpha, float beta) {
        illuminationChange_0(src.nativeObj, mask.nativeObj, dst.nativeObj, alpha, beta);
    }

    public static void illuminationChange(Mat src, Mat mask, Mat dst, float alpha) {
        illuminationChange_1(src.nativeObj, mask.nativeObj, dst.nativeObj, alpha);
    }

    public static void illuminationChange(Mat src, Mat mask, Mat dst) {
        illuminationChange_2(src.nativeObj, mask.nativeObj, dst.nativeObj);
    }

    public static void inpaint(Mat src, Mat inpaintMask, Mat dst, double inpaintRadius, int flags) {
        inpaint_0(src.nativeObj, inpaintMask.nativeObj, dst.nativeObj, inpaintRadius, flags);
    }

    public static void pencilSketch(Mat src, Mat dst1, Mat dst2, float sigma_s, float sigma_r, float shade_factor) {
        pencilSketch_0(src.nativeObj, dst1.nativeObj, dst2.nativeObj, sigma_s, sigma_r, shade_factor);
    }

    public static void pencilSketch(Mat src, Mat dst1, Mat dst2, float sigma_s, float sigma_r) {
        pencilSketch_1(src.nativeObj, dst1.nativeObj, dst2.nativeObj, sigma_s, sigma_r);
    }

    public static void pencilSketch(Mat src, Mat dst1, Mat dst2, float sigma_s) {
        pencilSketch_2(src.nativeObj, dst1.nativeObj, dst2.nativeObj, sigma_s);
    }

    public static void pencilSketch(Mat src, Mat dst1, Mat dst2) {
        pencilSketch_3(src.nativeObj, dst1.nativeObj, dst2.nativeObj);
    }

    public static void seamlessClone(Mat src, Mat dst, Mat mask, Point p, Mat blend, int flags) {
        Point point = p;
        seamlessClone_0(src.nativeObj, dst.nativeObj, mask.nativeObj, point.x, point.y, blend.nativeObj, flags);
    }

    public static void stylization(Mat src, Mat dst, float sigma_s, float sigma_r) {
        stylization_0(src.nativeObj, dst.nativeObj, sigma_s, sigma_r);
    }

    public static void stylization(Mat src, Mat dst, float sigma_s) {
        stylization_1(src.nativeObj, dst.nativeObj, sigma_s);
    }

    public static void stylization(Mat src, Mat dst) {
        stylization_2(src.nativeObj, dst.nativeObj);
    }

    public static void textureFlattening(Mat src, Mat mask, Mat dst, float low_threshold, float high_threshold, int kernel_size) {
        textureFlattening_0(src.nativeObj, mask.nativeObj, dst.nativeObj, low_threshold, high_threshold, kernel_size);
    }

    public static void textureFlattening(Mat src, Mat mask, Mat dst, float low_threshold, float high_threshold) {
        textureFlattening_1(src.nativeObj, mask.nativeObj, dst.nativeObj, low_threshold, high_threshold);
    }

    public static void textureFlattening(Mat src, Mat mask, Mat dst, float low_threshold) {
        textureFlattening_2(src.nativeObj, mask.nativeObj, dst.nativeObj, low_threshold);
    }

    public static void textureFlattening(Mat src, Mat mask, Mat dst) {
        textureFlattening_3(src.nativeObj, mask.nativeObj, dst.nativeObj);
    }
}
