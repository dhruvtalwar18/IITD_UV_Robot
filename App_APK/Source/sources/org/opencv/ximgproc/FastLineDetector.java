package org.opencv.ximgproc;

import org.opencv.core.Algorithm;
import org.opencv.core.Mat;

public class FastLineDetector extends Algorithm {
    private static native void delete(long j);

    private static native void detect_0(long j, long j2, long j3);

    private static native void drawSegments_0(long j, long j2, long j3, boolean z);

    private static native void drawSegments_1(long j, long j2, long j3);

    protected FastLineDetector(long addr) {
        super(addr);
    }

    public static FastLineDetector __fromPtr__(long addr) {
        return new FastLineDetector(addr);
    }

    public void detect(Mat _image, Mat _lines) {
        detect_0(this.nativeObj, _image.nativeObj, _lines.nativeObj);
    }

    public void drawSegments(Mat _image, Mat lines, boolean draw_arrow) {
        drawSegments_0(this.nativeObj, _image.nativeObj, lines.nativeObj, draw_arrow);
    }

    public void drawSegments(Mat _image, Mat lines) {
        drawSegments_1(this.nativeObj, _image.nativeObj, lines.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
