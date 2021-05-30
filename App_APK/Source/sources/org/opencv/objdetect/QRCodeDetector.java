package org.opencv.objdetect;

import org.opencv.core.Mat;

public class QRCodeDetector {
    protected final long nativeObj;

    private static native long QRCodeDetector_0();

    private static native void delete(long j);

    private static native boolean detect_0(long j, long j2, long j3);

    private static native void setEpsX_0(long j, double d);

    private static native void setEpsY_0(long j, double d);

    protected QRCodeDetector(long addr) {
        this.nativeObj = addr;
    }

    public long getNativeObjAddr() {
        return this.nativeObj;
    }

    public static QRCodeDetector __fromPtr__(long addr) {
        return new QRCodeDetector(addr);
    }

    public QRCodeDetector() {
        this.nativeObj = QRCodeDetector_0();
    }

    public boolean detect(Mat img, Mat points) {
        return detect_0(this.nativeObj, img.nativeObj, points.nativeObj);
    }

    public void setEpsX(double epsX) {
        setEpsX_0(this.nativeObj, epsX);
    }

    public void setEpsY(double epsY) {
        setEpsY_0(this.nativeObj, epsY);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
