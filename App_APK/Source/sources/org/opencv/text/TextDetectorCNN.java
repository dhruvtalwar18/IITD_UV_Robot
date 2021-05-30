package org.opencv.text;

import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfRect;

public class TextDetectorCNN extends TextDetector {
    private static native long create_0(String str, String str2);

    private static native void delete(long j);

    private static native void detect_0(long j, long j2, long j3, long j4);

    protected TextDetectorCNN(long addr) {
        super(addr);
    }

    public static TextDetectorCNN __fromPtr__(long addr) {
        return new TextDetectorCNN(addr);
    }

    public static TextDetectorCNN create(String modelArchFilename, String modelWeightsFilename) {
        return __fromPtr__(create_0(modelArchFilename, modelWeightsFilename));
    }

    public void detect(Mat inputImage, MatOfRect Bbox, MatOfFloat confidence) {
        detect_0(this.nativeObj, inputImage.nativeObj, Bbox.nativeObj, confidence.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
