package org.opencv.xfeatures2d;

import java.util.List;
import org.opencv.core.Algorithm;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.utils.Converters;

public class PCTSignaturesSQFD extends Algorithm {
    private static native float computeQuadraticFormDistance_0(long j, long j2, long j3);

    private static native void computeQuadraticFormDistances_0(long j, long j2, long j3, long j4);

    private static native long create_0(int i, int i2, float f);

    private static native long create_1(int i, int i2);

    private static native long create_2(int i);

    private static native long create_3();

    private static native void delete(long j);

    protected PCTSignaturesSQFD(long addr) {
        super(addr);
    }

    public static PCTSignaturesSQFD __fromPtr__(long addr) {
        return new PCTSignaturesSQFD(addr);
    }

    public static PCTSignaturesSQFD create(int distanceFunction, int similarityFunction, float similarityParameter) {
        return __fromPtr__(create_0(distanceFunction, similarityFunction, similarityParameter));
    }

    public static PCTSignaturesSQFD create(int distanceFunction, int similarityFunction) {
        return __fromPtr__(create_1(distanceFunction, similarityFunction));
    }

    public static PCTSignaturesSQFD create(int distanceFunction) {
        return __fromPtr__(create_2(distanceFunction));
    }

    public static PCTSignaturesSQFD create() {
        return __fromPtr__(create_3());
    }

    public float computeQuadraticFormDistance(Mat _signature0, Mat _signature1) {
        return computeQuadraticFormDistance_0(this.nativeObj, _signature0.nativeObj, _signature1.nativeObj);
    }

    public void computeQuadraticFormDistances(Mat sourceSignature, List<Mat> imageSignatures, MatOfFloat distances) {
        computeQuadraticFormDistances_0(this.nativeObj, sourceSignature.nativeObj, Converters.vector_Mat_to_Mat(imageSignatures).nativeObj, distances.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
