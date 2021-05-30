package org.opencv.img_hash;

public class RadialVarianceHash extends ImgHashBase {
    private static native long create_0(double d, int i);

    private static native long create_1(double d);

    private static native long create_2();

    private static native void delete(long j);

    private static native int getNumOfAngleLine_0(long j);

    private static native double getSigma_0(long j);

    private static native void setNumOfAngleLine_0(long j, int i);

    private static native void setSigma_0(long j, double d);

    protected RadialVarianceHash(long addr) {
        super(addr);
    }

    public static RadialVarianceHash __fromPtr__(long addr) {
        return new RadialVarianceHash(addr);
    }

    public static RadialVarianceHash create(double sigma, int numOfAngleLine) {
        return __fromPtr__(create_0(sigma, numOfAngleLine));
    }

    public static RadialVarianceHash create(double sigma) {
        return __fromPtr__(create_1(sigma));
    }

    public static RadialVarianceHash create() {
        return __fromPtr__(create_2());
    }

    public double getSigma() {
        return getSigma_0(this.nativeObj);
    }

    public int getNumOfAngleLine() {
        return getNumOfAngleLine_0(this.nativeObj);
    }

    public void setNumOfAngleLine(int value) {
        setNumOfAngleLine_0(this.nativeObj, value);
    }

    public void setSigma(double value) {
        setSigma_0(this.nativeObj, value);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
