package org.opencv.bioinspired;

import org.opencv.core.Algorithm;
import org.opencv.core.Mat;
import org.opencv.core.Size;

public class TransientAreasSegmentationModule extends Algorithm {
    private static native void clearAllBuffers_0(long j);

    private static native long create_0(double d, double d2);

    private static native void delete(long j);

    private static native void getSegmentationPicture_0(long j, long j2);

    private static native double[] getSize_0(long j);

    private static native String printSetup_0(long j);

    private static native void run_0(long j, long j2, int i);

    private static native void run_1(long j, long j2);

    private static native void setup_0(long j, String str, boolean z);

    private static native void setup_1(long j, String str);

    private static native void setup_2(long j);

    private static native void write_0(long j, String str);

    protected TransientAreasSegmentationModule(long addr) {
        super(addr);
    }

    public static TransientAreasSegmentationModule __fromPtr__(long addr) {
        return new TransientAreasSegmentationModule(addr);
    }

    public static TransientAreasSegmentationModule create(Size inputSize) {
        return __fromPtr__(create_0(inputSize.width, inputSize.height));
    }

    public Size getSize() {
        return new Size(getSize_0(this.nativeObj));
    }

    public String printSetup() {
        return printSetup_0(this.nativeObj);
    }

    public void clearAllBuffers() {
        clearAllBuffers_0(this.nativeObj);
    }

    public void getSegmentationPicture(Mat transientAreas) {
        getSegmentationPicture_0(this.nativeObj, transientAreas.nativeObj);
    }

    public void run(Mat inputToSegment, int channelIndex) {
        run_0(this.nativeObj, inputToSegment.nativeObj, channelIndex);
    }

    public void run(Mat inputToSegment) {
        run_1(this.nativeObj, inputToSegment.nativeObj);
    }

    public void setup(String segmentationParameterFile, boolean applyDefaultSetupOnFailure) {
        setup_0(this.nativeObj, segmentationParameterFile, applyDefaultSetupOnFailure);
    }

    public void setup(String segmentationParameterFile) {
        setup_1(this.nativeObj, segmentationParameterFile);
    }

    public void setup() {
        setup_2(this.nativeObj);
    }

    public void write(String fs) {
        write_0(this.nativeObj, fs);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
