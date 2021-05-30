package org.opencv.img_hash;

import org.opencv.core.Algorithm;
import org.opencv.core.Mat;

public class ImgHashBase extends Algorithm {
    private static native double compare_0(long j, long j2, long j3);

    private static native void compute_0(long j, long j2, long j3);

    private static native void delete(long j);

    protected ImgHashBase(long addr) {
        super(addr);
    }

    public static ImgHashBase __fromPtr__(long addr) {
        return new ImgHashBase(addr);
    }

    public double compare(Mat hashOne, Mat hashTwo) {
        return compare_0(this.nativeObj, hashOne.nativeObj, hashTwo.nativeObj);
    }

    public void compute(Mat inputArr, Mat outputArr) {
        compute_0(this.nativeObj, inputArr.nativeObj, outputArr.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
