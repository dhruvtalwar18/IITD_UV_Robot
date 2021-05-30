package org.opencv.features2d;

public class GFTTDetector extends Feature2D {
    private static native long create_0(int i, double d, double d2, int i2, int i3, boolean z, double d3);

    private static native long create_1(int i, double d, double d2, int i2, int i3, boolean z);

    private static native long create_2(int i, double d, double d2, int i2, int i3);

    private static native long create_3(int i, double d, double d2, int i2, boolean z, double d3);

    private static native long create_4(int i, double d, double d2, int i2, boolean z);

    private static native long create_5(int i, double d, double d2, int i2);

    private static native long create_6(int i, double d, double d2);

    private static native long create_7(int i, double d);

    private static native long create_8(int i);

    private static native long create_9();

    private static native void delete(long j);

    private static native int getBlockSize_0(long j);

    private static native String getDefaultName_0(long j);

    private static native boolean getHarrisDetector_0(long j);

    private static native double getK_0(long j);

    private static native int getMaxFeatures_0(long j);

    private static native double getMinDistance_0(long j);

    private static native double getQualityLevel_0(long j);

    private static native void setBlockSize_0(long j, int i);

    private static native void setHarrisDetector_0(long j, boolean z);

    private static native void setK_0(long j, double d);

    private static native void setMaxFeatures_0(long j, int i);

    private static native void setMinDistance_0(long j, double d);

    private static native void setQualityLevel_0(long j, double d);

    protected GFTTDetector(long addr) {
        super(addr);
    }

    public static GFTTDetector __fromPtr__(long addr) {
        return new GFTTDetector(addr);
    }

    public static GFTTDetector create(int maxCorners, double qualityLevel, double minDistance, int blockSize, int gradiantSize, boolean useHarrisDetector, double k) {
        return __fromPtr__(create_0(maxCorners, qualityLevel, minDistance, blockSize, gradiantSize, useHarrisDetector, k));
    }

    public static GFTTDetector create(int maxCorners, double qualityLevel, double minDistance, int blockSize, int gradiantSize, boolean useHarrisDetector) {
        return __fromPtr__(create_1(maxCorners, qualityLevel, minDistance, blockSize, gradiantSize, useHarrisDetector));
    }

    public static GFTTDetector create(int maxCorners, double qualityLevel, double minDistance, int blockSize, int gradiantSize) {
        return __fromPtr__(create_2(maxCorners, qualityLevel, minDistance, blockSize, gradiantSize));
    }

    public static GFTTDetector create(int maxCorners, double qualityLevel, double minDistance, int blockSize, boolean useHarrisDetector, double k) {
        return __fromPtr__(create_3(maxCorners, qualityLevel, minDistance, blockSize, useHarrisDetector, k));
    }

    public static GFTTDetector create(int maxCorners, double qualityLevel, double minDistance, int blockSize, boolean useHarrisDetector) {
        return __fromPtr__(create_4(maxCorners, qualityLevel, minDistance, blockSize, useHarrisDetector));
    }

    public static GFTTDetector create(int maxCorners, double qualityLevel, double minDistance, int blockSize) {
        return __fromPtr__(create_5(maxCorners, qualityLevel, minDistance, blockSize));
    }

    public static GFTTDetector create(int maxCorners, double qualityLevel, double minDistance) {
        return __fromPtr__(create_6(maxCorners, qualityLevel, minDistance));
    }

    public static GFTTDetector create(int maxCorners, double qualityLevel) {
        return __fromPtr__(create_7(maxCorners, qualityLevel));
    }

    public static GFTTDetector create(int maxCorners) {
        return __fromPtr__(create_8(maxCorners));
    }

    public static GFTTDetector create() {
        return __fromPtr__(create_9());
    }

    public String getDefaultName() {
        return getDefaultName_0(this.nativeObj);
    }

    public boolean getHarrisDetector() {
        return getHarrisDetector_0(this.nativeObj);
    }

    public double getK() {
        return getK_0(this.nativeObj);
    }

    public double getMinDistance() {
        return getMinDistance_0(this.nativeObj);
    }

    public double getQualityLevel() {
        return getQualityLevel_0(this.nativeObj);
    }

    public int getBlockSize() {
        return getBlockSize_0(this.nativeObj);
    }

    public int getMaxFeatures() {
        return getMaxFeatures_0(this.nativeObj);
    }

    public void setBlockSize(int blockSize) {
        setBlockSize_0(this.nativeObj, blockSize);
    }

    public void setHarrisDetector(boolean val) {
        setHarrisDetector_0(this.nativeObj, val);
    }

    public void setK(double k) {
        setK_0(this.nativeObj, k);
    }

    public void setMaxFeatures(int maxFeatures) {
        setMaxFeatures_0(this.nativeObj, maxFeatures);
    }

    public void setMinDistance(double minDistance) {
        setMinDistance_0(this.nativeObj, minDistance);
    }

    public void setQualityLevel(double qlevel) {
        setQualityLevel_0(this.nativeObj, qlevel);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
