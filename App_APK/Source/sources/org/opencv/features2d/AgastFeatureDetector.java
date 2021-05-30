package org.opencv.features2d;

public class AgastFeatureDetector extends Feature2D {
    public static final int AGAST_5_8 = 0;
    public static final int AGAST_7_12d = 1;
    public static final int AGAST_7_12s = 2;
    public static final int NONMAX_SUPPRESSION = 10001;
    public static final int OAST_9_16 = 3;
    public static final int THRESHOLD = 10000;

    private static native long create_0(int i, boolean z, int i2);

    private static native long create_1(int i, boolean z);

    private static native long create_2(int i);

    private static native long create_3();

    private static native void delete(long j);

    private static native String getDefaultName_0(long j);

    private static native boolean getNonmaxSuppression_0(long j);

    private static native int getThreshold_0(long j);

    private static native int getType_0(long j);

    private static native void setNonmaxSuppression_0(long j, boolean z);

    private static native void setThreshold_0(long j, int i);

    private static native void setType_0(long j, int i);

    protected AgastFeatureDetector(long addr) {
        super(addr);
    }

    public static AgastFeatureDetector __fromPtr__(long addr) {
        return new AgastFeatureDetector(addr);
    }

    public int getType() {
        return getType_0(this.nativeObj);
    }

    public static AgastFeatureDetector create(int threshold, boolean nonmaxSuppression, int type) {
        return __fromPtr__(create_0(threshold, nonmaxSuppression, type));
    }

    public static AgastFeatureDetector create(int threshold, boolean nonmaxSuppression) {
        return __fromPtr__(create_1(threshold, nonmaxSuppression));
    }

    public static AgastFeatureDetector create(int threshold) {
        return __fromPtr__(create_2(threshold));
    }

    public static AgastFeatureDetector create() {
        return __fromPtr__(create_3());
    }

    public String getDefaultName() {
        return getDefaultName_0(this.nativeObj);
    }

    public boolean getNonmaxSuppression() {
        return getNonmaxSuppression_0(this.nativeObj);
    }

    public int getThreshold() {
        return getThreshold_0(this.nativeObj);
    }

    public void setNonmaxSuppression(boolean f) {
        setNonmaxSuppression_0(this.nativeObj, f);
    }

    public void setThreshold(int threshold) {
        setThreshold_0(this.nativeObj, threshold);
    }

    public void setType(int type) {
        setType_0(this.nativeObj, type);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
