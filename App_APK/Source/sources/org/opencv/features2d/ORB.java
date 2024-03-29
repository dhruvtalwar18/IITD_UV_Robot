package org.opencv.features2d;

public class ORB extends Feature2D {
    public static final int FAST_SCORE = 1;
    public static final int HARRIS_SCORE = 0;

    private static native long create_0(int i, float f, int i2, int i3, int i4, int i5, int i6, int i7, int i8);

    private static native long create_1(int i, float f, int i2, int i3, int i4, int i5, int i6, int i7);

    private static native long create_2(int i, float f, int i2, int i3, int i4, int i5, int i6);

    private static native long create_3(int i, float f, int i2, int i3, int i4, int i5);

    private static native long create_4(int i, float f, int i2, int i3, int i4);

    private static native long create_5(int i, float f, int i2, int i3);

    private static native long create_6(int i, float f, int i2);

    private static native long create_7(int i, float f);

    private static native long create_8(int i);

    private static native long create_9();

    private static native void delete(long j);

    private static native String getDefaultName_0(long j);

    private static native int getEdgeThreshold_0(long j);

    private static native int getFastThreshold_0(long j);

    private static native int getFirstLevel_0(long j);

    private static native int getMaxFeatures_0(long j);

    private static native int getNLevels_0(long j);

    private static native int getPatchSize_0(long j);

    private static native double getScaleFactor_0(long j);

    private static native int getScoreType_0(long j);

    private static native int getWTA_K_0(long j);

    private static native void setEdgeThreshold_0(long j, int i);

    private static native void setFastThreshold_0(long j, int i);

    private static native void setFirstLevel_0(long j, int i);

    private static native void setMaxFeatures_0(long j, int i);

    private static native void setNLevels_0(long j, int i);

    private static native void setPatchSize_0(long j, int i);

    private static native void setScaleFactor_0(long j, double d);

    private static native void setScoreType_0(long j, int i);

    private static native void setWTA_K_0(long j, int i);

    protected ORB(long addr) {
        super(addr);
    }

    public static ORB __fromPtr__(long addr) {
        return new ORB(addr);
    }

    public int getScoreType() {
        return getScoreType_0(this.nativeObj);
    }

    public static ORB create(int nfeatures, float scaleFactor, int nlevels, int edgeThreshold, int firstLevel, int WTA_K, int scoreType, int patchSize, int fastThreshold) {
        return __fromPtr__(create_0(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize, fastThreshold));
    }

    public static ORB create(int nfeatures, float scaleFactor, int nlevels, int edgeThreshold, int firstLevel, int WTA_K, int scoreType, int patchSize) {
        return __fromPtr__(create_1(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize));
    }

    public static ORB create(int nfeatures, float scaleFactor, int nlevels, int edgeThreshold, int firstLevel, int WTA_K, int scoreType) {
        return __fromPtr__(create_2(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType));
    }

    public static ORB create(int nfeatures, float scaleFactor, int nlevels, int edgeThreshold, int firstLevel, int WTA_K) {
        return __fromPtr__(create_3(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K));
    }

    public static ORB create(int nfeatures, float scaleFactor, int nlevels, int edgeThreshold, int firstLevel) {
        return __fromPtr__(create_4(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel));
    }

    public static ORB create(int nfeatures, float scaleFactor, int nlevels, int edgeThreshold) {
        return __fromPtr__(create_5(nfeatures, scaleFactor, nlevels, edgeThreshold));
    }

    public static ORB create(int nfeatures, float scaleFactor, int nlevels) {
        return __fromPtr__(create_6(nfeatures, scaleFactor, nlevels));
    }

    public static ORB create(int nfeatures, float scaleFactor) {
        return __fromPtr__(create_7(nfeatures, scaleFactor));
    }

    public static ORB create(int nfeatures) {
        return __fromPtr__(create_8(nfeatures));
    }

    public static ORB create() {
        return __fromPtr__(create_9());
    }

    public String getDefaultName() {
        return getDefaultName_0(this.nativeObj);
    }

    public double getScaleFactor() {
        return getScaleFactor_0(this.nativeObj);
    }

    public int getEdgeThreshold() {
        return getEdgeThreshold_0(this.nativeObj);
    }

    public int getFastThreshold() {
        return getFastThreshold_0(this.nativeObj);
    }

    public int getFirstLevel() {
        return getFirstLevel_0(this.nativeObj);
    }

    public int getMaxFeatures() {
        return getMaxFeatures_0(this.nativeObj);
    }

    public int getNLevels() {
        return getNLevels_0(this.nativeObj);
    }

    public int getPatchSize() {
        return getPatchSize_0(this.nativeObj);
    }

    public int getWTA_K() {
        return getWTA_K_0(this.nativeObj);
    }

    public void setEdgeThreshold(int edgeThreshold) {
        setEdgeThreshold_0(this.nativeObj, edgeThreshold);
    }

    public void setFastThreshold(int fastThreshold) {
        setFastThreshold_0(this.nativeObj, fastThreshold);
    }

    public void setFirstLevel(int firstLevel) {
        setFirstLevel_0(this.nativeObj, firstLevel);
    }

    public void setMaxFeatures(int maxFeatures) {
        setMaxFeatures_0(this.nativeObj, maxFeatures);
    }

    public void setNLevels(int nlevels) {
        setNLevels_0(this.nativeObj, nlevels);
    }

    public void setPatchSize(int patchSize) {
        setPatchSize_0(this.nativeObj, patchSize);
    }

    public void setScaleFactor(double scaleFactor) {
        setScaleFactor_0(this.nativeObj, scaleFactor);
    }

    public void setScoreType(int scoreType) {
        setScoreType_0(this.nativeObj, scoreType);
    }

    public void setWTA_K(int wta_k) {
        setWTA_K_0(this.nativeObj, wta_k);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
