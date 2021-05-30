package org.opencv.aruco;

import org.opencv.core.Mat;

public class Dictionary {
    protected final long nativeObj;

    private static native long create_0(int i, int i2, int i3);

    private static native long create_1(int i, int i2);

    private static native long create_from_0(int i, int i2, long j, int i3);

    private static native long create_from_1(int i, int i2, long j);

    private static native void delete(long j);

    private static native void drawMarker_0(long j, int i, int i2, long j2, int i3);

    private static native void drawMarker_1(long j, int i, int i2, long j2);

    private static native long getBitsFromByteList_0(long j, int i);

    private static native long getByteListFromBits_0(long j);

    private static native long get_0(int i);

    private static native long get_bytesList_0(long j);

    private static native int get_markerSize_0(long j);

    private static native int get_maxCorrectionBits_0(long j);

    private static native void set_bytesList_0(long j, long j2);

    private static native void set_markerSize_0(long j, int i);

    private static native void set_maxCorrectionBits_0(long j, int i);

    protected Dictionary(long addr) {
        this.nativeObj = addr;
    }

    public long getNativeObjAddr() {
        return this.nativeObj;
    }

    public static Dictionary __fromPtr__(long addr) {
        return new Dictionary(addr);
    }

    public static Mat getBitsFromByteList(Mat byteList, int markerSize) {
        return new Mat(getBitsFromByteList_0(byteList.nativeObj, markerSize));
    }

    public static Mat getByteListFromBits(Mat bits) {
        return new Mat(getByteListFromBits_0(bits.nativeObj));
    }

    public static Dictionary create_from(int nMarkers, int markerSize, Dictionary baseDictionary, int randomSeed) {
        return __fromPtr__(create_from_0(nMarkers, markerSize, baseDictionary.getNativeObjAddr(), randomSeed));
    }

    public static Dictionary create_from(int nMarkers, int markerSize, Dictionary baseDictionary) {
        return __fromPtr__(create_from_1(nMarkers, markerSize, baseDictionary.getNativeObjAddr()));
    }

    public static Dictionary create(int nMarkers, int markerSize, int randomSeed) {
        return __fromPtr__(create_0(nMarkers, markerSize, randomSeed));
    }

    public static Dictionary create(int nMarkers, int markerSize) {
        return __fromPtr__(create_1(nMarkers, markerSize));
    }

    public static Dictionary get(int dict) {
        return __fromPtr__(get_0(dict));
    }

    public void drawMarker(int id, int sidePixels, Mat _img, int borderBits) {
        drawMarker_0(this.nativeObj, id, sidePixels, _img.nativeObj, borderBits);
    }

    public void drawMarker(int id, int sidePixels, Mat _img) {
        drawMarker_1(this.nativeObj, id, sidePixels, _img.nativeObj);
    }

    public Mat get_bytesList() {
        return new Mat(get_bytesList_0(this.nativeObj));
    }

    public void set_bytesList(Mat bytesList) {
        set_bytesList_0(this.nativeObj, bytesList.nativeObj);
    }

    public int get_markerSize() {
        return get_markerSize_0(this.nativeObj);
    }

    public void set_markerSize(int markerSize) {
        set_markerSize_0(this.nativeObj, markerSize);
    }

    public int get_maxCorrectionBits() {
        return get_maxCorrectionBits_0(this.nativeObj);
    }

    public void set_maxCorrectionBits(int maxCorrectionBits) {
        set_maxCorrectionBits_0(this.nativeObj, maxCorrectionBits);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
