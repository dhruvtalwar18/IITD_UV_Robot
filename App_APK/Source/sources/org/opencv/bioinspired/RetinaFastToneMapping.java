package org.opencv.bioinspired;

import org.opencv.core.Algorithm;
import org.opencv.core.Mat;
import org.opencv.core.Size;

public class RetinaFastToneMapping extends Algorithm {
    private static native void applyFastToneMapping_0(long j, long j2, long j3);

    private static native long create_0(double d, double d2);

    private static native void delete(long j);

    private static native void setup_0(long j, float f, float f2, float f3);

    private static native void setup_1(long j, float f, float f2);

    private static native void setup_2(long j, float f);

    private static native void setup_3(long j);

    protected RetinaFastToneMapping(long addr) {
        super(addr);
    }

    public static RetinaFastToneMapping __fromPtr__(long addr) {
        return new RetinaFastToneMapping(addr);
    }

    public static RetinaFastToneMapping create(Size inputSize) {
        return __fromPtr__(create_0(inputSize.width, inputSize.height));
    }

    public void applyFastToneMapping(Mat inputImage, Mat outputToneMappedImage) {
        applyFastToneMapping_0(this.nativeObj, inputImage.nativeObj, outputToneMappedImage.nativeObj);
    }

    public void setup(float photoreceptorsNeighborhoodRadius, float ganglioncellsNeighborhoodRadius, float meanLuminanceModulatorK) {
        setup_0(this.nativeObj, photoreceptorsNeighborhoodRadius, ganglioncellsNeighborhoodRadius, meanLuminanceModulatorK);
    }

    public void setup(float photoreceptorsNeighborhoodRadius, float ganglioncellsNeighborhoodRadius) {
        setup_1(this.nativeObj, photoreceptorsNeighborhoodRadius, ganglioncellsNeighborhoodRadius);
    }

    public void setup(float photoreceptorsNeighborhoodRadius) {
        setup_2(this.nativeObj, photoreceptorsNeighborhoodRadius);
    }

    public void setup() {
        setup_3(this.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
