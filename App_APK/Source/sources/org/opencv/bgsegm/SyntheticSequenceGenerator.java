package org.opencv.bgsegm;

import org.opencv.core.Algorithm;
import org.opencv.core.Mat;

public class SyntheticSequenceGenerator extends Algorithm {
    private static native long SyntheticSequenceGenerator_0(long j, long j2, double d, double d2, double d3, double d4);

    private static native void delete(long j);

    private static native void getNextFrame_0(long j, long j2, long j3);

    protected SyntheticSequenceGenerator(long addr) {
        super(addr);
    }

    public static SyntheticSequenceGenerator __fromPtr__(long addr) {
        return new SyntheticSequenceGenerator(addr);
    }

    public SyntheticSequenceGenerator(Mat background, Mat object, double amplitude, double wavelength, double wavespeed, double objspeed) {
        super(SyntheticSequenceGenerator_0(background.nativeObj, object.nativeObj, amplitude, wavelength, wavespeed, objspeed));
    }

    public void getNextFrame(Mat frame, Mat gtMask) {
        getNextFrame_0(this.nativeObj, frame.nativeObj, gtMask.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
