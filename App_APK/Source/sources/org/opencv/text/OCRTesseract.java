package org.opencv.text;

import org.opencv.core.Mat;

public class OCRTesseract extends BaseOCR {
    private static native long create_0(String str, String str2, String str3, int i, int i2);

    private static native long create_1(String str, String str2, String str3, int i);

    private static native long create_2(String str, String str2, String str3);

    private static native long create_3(String str, String str2);

    private static native long create_4(String str);

    private static native long create_5();

    private static native void delete(long j);

    private static native String run_0(long j, long j2, long j3, int i, int i2);

    private static native String run_1(long j, long j2, long j3, int i);

    private static native String run_2(long j, long j2, int i, int i2);

    private static native String run_3(long j, long j2, int i);

    private static native void setWhiteList_0(long j, String str);

    protected OCRTesseract(long addr) {
        super(addr);
    }

    public static OCRTesseract __fromPtr__(long addr) {
        return new OCRTesseract(addr);
    }

    public static OCRTesseract create(String datapath, String language, String char_whitelist, int oem, int psmode) {
        return __fromPtr__(create_0(datapath, language, char_whitelist, oem, psmode));
    }

    public static OCRTesseract create(String datapath, String language, String char_whitelist, int oem) {
        return __fromPtr__(create_1(datapath, language, char_whitelist, oem));
    }

    public static OCRTesseract create(String datapath, String language, String char_whitelist) {
        return __fromPtr__(create_2(datapath, language, char_whitelist));
    }

    public static OCRTesseract create(String datapath, String language) {
        return __fromPtr__(create_3(datapath, language));
    }

    public static OCRTesseract create(String datapath) {
        return __fromPtr__(create_4(datapath));
    }

    public static OCRTesseract create() {
        return __fromPtr__(create_5());
    }

    public String run(Mat image, Mat mask, int min_confidence, int component_level) {
        return run_0(this.nativeObj, image.nativeObj, mask.nativeObj, min_confidence, component_level);
    }

    public String run(Mat image, Mat mask, int min_confidence) {
        return run_1(this.nativeObj, image.nativeObj, mask.nativeObj, min_confidence);
    }

    public String run(Mat image, int min_confidence, int component_level) {
        return run_2(this.nativeObj, image.nativeObj, min_confidence, component_level);
    }

    public String run(Mat image, int min_confidence) {
        return run_3(this.nativeObj, image.nativeObj, min_confidence);
    }

    public void setWhiteList(String char_whitelist) {
        setWhiteList_0(this.nativeObj, char_whitelist);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
