package org.opencv.text;

import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfRect;

public class TextDetector {
    protected final long nativeObj;

    private static native void delete(long j);

    private static native void detect_0(long j, long j2, long j3, long j4);

    protected TextDetector(long addr) {
        this.nativeObj = addr;
    }

    public long getNativeObjAddr() {
        return this.nativeObj;
    }

    public static TextDetector __fromPtr__(long addr) {
        return new TextDetector(addr);
    }

    public void detect(Mat inputImage, MatOfRect Bbox, MatOfFloat confidence) {
        detect_0(this.nativeObj, inputImage.nativeObj, Bbox.nativeObj, confidence.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
