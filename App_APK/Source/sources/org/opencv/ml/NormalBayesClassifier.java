package org.opencv.ml;

import org.opencv.core.Mat;

public class NormalBayesClassifier extends StatModel {
    private static native long create_0();

    private static native void delete(long j);

    private static native long load_0(String str, String str2);

    private static native long load_1(String str);

    private static native float predictProb_0(long j, long j2, long j3, long j4, int i);

    private static native float predictProb_1(long j, long j2, long j3, long j4);

    protected NormalBayesClassifier(long addr) {
        super(addr);
    }

    public static NormalBayesClassifier __fromPtr__(long addr) {
        return new NormalBayesClassifier(addr);
    }

    public static NormalBayesClassifier create() {
        return __fromPtr__(create_0());
    }

    public static NormalBayesClassifier load(String filepath, String nodeName) {
        return __fromPtr__(load_0(filepath, nodeName));
    }

    public static NormalBayesClassifier load(String filepath) {
        return __fromPtr__(load_1(filepath));
    }

    public float predictProb(Mat inputs, Mat outputs, Mat outputProbs, int flags) {
        return predictProb_0(this.nativeObj, inputs.nativeObj, outputs.nativeObj, outputProbs.nativeObj, flags);
    }

    public float predictProb(Mat inputs, Mat outputs, Mat outputProbs) {
        return predictProb_1(this.nativeObj, inputs.nativeObj, outputs.nativeObj, outputProbs.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
