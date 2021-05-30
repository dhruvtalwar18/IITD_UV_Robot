package org.opencv.objdetect;

import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfRect;
import org.opencv.core.Size;

public class HOGDescriptor {
    public static final int DEFAULT_NLEVELS = 64;
    public static final int DESCR_FORMAT_COL_BY_COL = 0;
    public static final int DESCR_FORMAT_ROW_BY_ROW = 1;
    public static final int L2Hys = 0;
    protected final long nativeObj;

    private static native long HOGDescriptor_0(double d, double d2, double d3, double d4, double d5, double d6, double d7, double d8, int i, int i2, double d9, int i3, double d10, boolean z, int i4, boolean z2);

    private static native long HOGDescriptor_1(double d, double d2, double d3, double d4, double d5, double d6, double d7, double d8, int i, int i2, double d9, int i3, double d10, boolean z, int i4);

    private static native long HOGDescriptor_2(double d, double d2, double d3, double d4, double d5, double d6, double d7, double d8, int i, int i2, double d9, int i3, double d10, boolean z);

    private static native long HOGDescriptor_3(double d, double d2, double d3, double d4, double d5, double d6, double d7, double d8, int i, int i2, double d9, int i3, double d10);

    private static native long HOGDescriptor_4(double d, double d2, double d3, double d4, double d5, double d6, double d7, double d8, int i, int i2, double d9, int i3);

    private static native long HOGDescriptor_5(double d, double d2, double d3, double d4, double d5, double d6, double d7, double d8, int i, int i2, double d9);

    private static native long HOGDescriptor_6(double d, double d2, double d3, double d4, double d5, double d6, double d7, double d8, int i, int i2);

    private static native long HOGDescriptor_7(double d, double d2, double d3, double d4, double d5, double d6, double d7, double d8, int i);

    private static native long HOGDescriptor_8(String str);

    private static native long HOGDescriptor_9();

    private static native boolean checkDetectorSize_0(long j);

    private static native void computeGradient_0(long j, long j2, long j3, long j4, double d, double d2, double d3, double d4);

    private static native void computeGradient_1(long j, long j2, long j3, long j4, double d, double d2);

    private static native void computeGradient_2(long j, long j2, long j3, long j4);

    private static native void compute_0(long j, long j2, long j3, double d, double d2, double d3, double d4, long j4);

    private static native void compute_1(long j, long j2, long j3, double d, double d2, double d3, double d4);

    private static native void compute_2(long j, long j2, long j3, double d, double d2);

    private static native void compute_3(long j, long j2, long j3);

    private static native void delete(long j);

    private static native void detectMultiScale_0(long j, long j2, long j3, long j4, double d, double d2, double d3, double d4, double d5, double d6, double d7, boolean z);

    private static native void detectMultiScale_1(long j, long j2, long j3, long j4, double d, double d2, double d3, double d4, double d5, double d6, double d7);

    private static native void detectMultiScale_2(long j, long j2, long j3, long j4, double d, double d2, double d3, double d4, double d5, double d6);

    private static native void detectMultiScale_3(long j, long j2, long j3, long j4, double d, double d2, double d3, double d4, double d5);

    private static native void detectMultiScale_4(long j, long j2, long j3, long j4, double d, double d2, double d3);

    private static native void detectMultiScale_5(long j, long j2, long j3, long j4, double d);

    private static native void detectMultiScale_6(long j, long j2, long j3, long j4);

    private static native void detect_0(long j, long j2, long j3, long j4, double d, double d2, double d3, double d4, double d5, long j5);

    private static native void detect_1(long j, long j2, long j3, long j4, double d, double d2, double d3, double d4, double d5);

    private static native void detect_2(long j, long j2, long j3, long j4, double d, double d2, double d3);

    private static native void detect_3(long j, long j2, long j3, long j4, double d);

    private static native void detect_4(long j, long j2, long j3, long j4);

    private static native long getDaimlerPeopleDetector_0();

    private static native long getDefaultPeopleDetector_0();

    private static native long getDescriptorSize_0(long j);

    private static native double getWinSigma_0(long j);

    private static native double get_L2HysThreshold_0(long j);

    private static native double[] get_blockSize_0(long j);

    private static native double[] get_blockStride_0(long j);

    private static native double[] get_cellSize_0(long j);

    private static native int get_derivAperture_0(long j);

    private static native boolean get_gammaCorrection_0(long j);

    private static native int get_histogramNormType_0(long j);

    private static native int get_nbins_0(long j);

    private static native int get_nlevels_0(long j);

    private static native boolean get_signedGradient_0(long j);

    private static native long get_svmDetector_0(long j);

    private static native double get_winSigma_0(long j);

    private static native double[] get_winSize_0(long j);

    private static native boolean load_0(long j, String str, String str2);

    private static native boolean load_1(long j, String str);

    private static native void save_0(long j, String str, String str2);

    private static native void save_1(long j, String str);

    private static native void setSVMDetector_0(long j, long j2);

    protected HOGDescriptor(long addr) {
        this.nativeObj = addr;
    }

    public long getNativeObjAddr() {
        return this.nativeObj;
    }

    public static HOGDescriptor __fromPtr__(long addr) {
        return new HOGDescriptor(addr);
    }

    public HOGDescriptor(Size _winSize, Size _blockSize, Size _blockStride, Size _cellSize, int _nbins, int _derivAperture, double _winSigma, int _histogramNormType, double _L2HysThreshold, boolean _gammaCorrection, int _nlevels, boolean _signedGradient) {
        Size size = _winSize;
        Size size2 = _blockSize;
        Size size3 = _blockStride;
        Size size4 = _cellSize;
        this.nativeObj = HOGDescriptor_0(size.width, size.height, size2.width, size2.height, size3.width, size3.height, size4.width, size4.height, _nbins, _derivAperture, _winSigma, _histogramNormType, _L2HysThreshold, _gammaCorrection, _nlevels, _signedGradient);
    }

    public HOGDescriptor(Size _winSize, Size _blockSize, Size _blockStride, Size _cellSize, int _nbins, int _derivAperture, double _winSigma, int _histogramNormType, double _L2HysThreshold, boolean _gammaCorrection, int _nlevels) {
        Size size = _winSize;
        Size size2 = _blockSize;
        Size size3 = _blockStride;
        Size size4 = _cellSize;
        this.nativeObj = HOGDescriptor_1(size.width, size.height, size2.width, size2.height, size3.width, size3.height, size4.width, size4.height, _nbins, _derivAperture, _winSigma, _histogramNormType, _L2HysThreshold, _gammaCorrection, _nlevels);
    }

    public HOGDescriptor(Size _winSize, Size _blockSize, Size _blockStride, Size _cellSize, int _nbins, int _derivAperture, double _winSigma, int _histogramNormType, double _L2HysThreshold, boolean _gammaCorrection) {
        Size size = _winSize;
        Size size2 = _blockSize;
        Size size3 = _blockStride;
        Size size4 = _cellSize;
        this.nativeObj = HOGDescriptor_2(size.width, size.height, size2.width, size2.height, size3.width, size3.height, size4.width, size4.height, _nbins, _derivAperture, _winSigma, _histogramNormType, _L2HysThreshold, _gammaCorrection);
    }

    public HOGDescriptor(Size _winSize, Size _blockSize, Size _blockStride, Size _cellSize, int _nbins, int _derivAperture, double _winSigma, int _histogramNormType, double _L2HysThreshold) {
        Size size = _winSize;
        Size size2 = _blockSize;
        Size size3 = _blockStride;
        Size size4 = _cellSize;
        this.nativeObj = HOGDescriptor_3(size.width, size.height, size2.width, size2.height, size3.width, size3.height, size4.width, size4.height, _nbins, _derivAperture, _winSigma, _histogramNormType, _L2HysThreshold);
    }

    public HOGDescriptor(Size _winSize, Size _blockSize, Size _blockStride, Size _cellSize, int _nbins, int _derivAperture, double _winSigma, int _histogramNormType) {
        Size size = _winSize;
        Size size2 = _blockSize;
        Size size3 = _blockStride;
        Size size4 = _cellSize;
        this.nativeObj = HOGDescriptor_4(size.width, size.height, size2.width, size2.height, size3.width, size3.height, size4.width, size4.height, _nbins, _derivAperture, _winSigma, _histogramNormType);
    }

    public HOGDescriptor(Size _winSize, Size _blockSize, Size _blockStride, Size _cellSize, int _nbins, int _derivAperture, double _winSigma) {
        Size size = _winSize;
        Size size2 = _blockSize;
        Size size3 = _blockStride;
        Size size4 = _cellSize;
        this.nativeObj = HOGDescriptor_5(size.width, size.height, size2.width, size2.height, size3.width, size3.height, size4.width, size4.height, _nbins, _derivAperture, _winSigma);
    }

    public HOGDescriptor(Size _winSize, Size _blockSize, Size _blockStride, Size _cellSize, int _nbins, int _derivAperture) {
        Size size = _winSize;
        Size size2 = _blockSize;
        Size size3 = _blockStride;
        Size size4 = _cellSize;
        this.nativeObj = HOGDescriptor_6(size.width, size.height, size2.width, size2.height, size3.width, size3.height, size4.width, size4.height, _nbins, _derivAperture);
    }

    public HOGDescriptor(Size _winSize, Size _blockSize, Size _blockStride, Size _cellSize, int _nbins) {
        Size size = _winSize;
        Size size2 = _blockSize;
        Size size3 = _blockStride;
        Size size4 = _cellSize;
        this.nativeObj = HOGDescriptor_7(size.width, size.height, size2.width, size2.height, size3.width, size3.height, size4.width, size4.height, _nbins);
    }

    public HOGDescriptor(String filename) {
        this.nativeObj = HOGDescriptor_8(filename);
    }

    public HOGDescriptor() {
        this.nativeObj = HOGDescriptor_9();
    }

    public boolean checkDetectorSize() {
        return checkDetectorSize_0(this.nativeObj);
    }

    public boolean load(String filename, String objname) {
        return load_0(this.nativeObj, filename, objname);
    }

    public boolean load(String filename) {
        return load_1(this.nativeObj, filename);
    }

    public double getWinSigma() {
        return getWinSigma_0(this.nativeObj);
    }

    public long getDescriptorSize() {
        return getDescriptorSize_0(this.nativeObj);
    }

    public static MatOfFloat getDaimlerPeopleDetector() {
        return MatOfFloat.fromNativeAddr(getDaimlerPeopleDetector_0());
    }

    public static MatOfFloat getDefaultPeopleDetector() {
        return MatOfFloat.fromNativeAddr(getDefaultPeopleDetector_0());
    }

    public void compute(Mat img, MatOfFloat descriptors, Size winStride, Size padding, MatOfPoint locations) {
        Size size = winStride;
        Size size2 = padding;
        compute_0(this.nativeObj, img.nativeObj, descriptors.nativeObj, size.width, size.height, size2.width, size2.height, locations.nativeObj);
    }

    public void compute(Mat img, MatOfFloat descriptors, Size winStride, Size padding) {
        Size size = winStride;
        Size size2 = padding;
        MatOfFloat matOfFloat = descriptors;
        MatOfFloat matOfFloat2 = matOfFloat;
        compute_1(this.nativeObj, img.nativeObj, matOfFloat.nativeObj, size.width, size.height, size2.width, size2.height);
    }

    public void compute(Mat img, MatOfFloat descriptors, Size winStride) {
        compute_2(this.nativeObj, img.nativeObj, descriptors.nativeObj, winStride.width, winStride.height);
    }

    public void compute(Mat img, MatOfFloat descriptors) {
        compute_3(this.nativeObj, img.nativeObj, descriptors.nativeObj);
    }

    public void computeGradient(Mat img, Mat grad, Mat angleOfs, Size paddingTL, Size paddingBR) {
        Size size = paddingTL;
        Size size2 = paddingBR;
        computeGradient_0(this.nativeObj, img.nativeObj, grad.nativeObj, angleOfs.nativeObj, size.width, size.height, size2.width, size2.height);
    }

    public void computeGradient(Mat img, Mat grad, Mat angleOfs, Size paddingTL) {
        Size size = paddingTL;
        computeGradient_1(this.nativeObj, img.nativeObj, grad.nativeObj, angleOfs.nativeObj, size.width, size.height);
    }

    public void computeGradient(Mat img, Mat grad, Mat angleOfs) {
        computeGradient_2(this.nativeObj, img.nativeObj, grad.nativeObj, angleOfs.nativeObj);
    }

    public void detect(Mat img, MatOfPoint foundLocations, MatOfDouble weights, double hitThreshold, Size winStride, Size padding, MatOfPoint searchLocations) {
        Size size = winStride;
        Size size2 = padding;
        MatOfPoint matOfPoint = foundLocations;
        Mat mat = weights;
        long j = this.nativeObj;
        long j2 = img.nativeObj;
        long j3 = j;
        MatOfPoint matOfPoint2 = searchLocations;
        MatOfPoint matOfPoint3 = matOfPoint;
        Mat foundLocations_mat = mat;
        long j4 = j2;
        long j5 = j3;
        MatOfPoint matOfPoint4 = matOfPoint2;
        long j6 = j5;
        detect_0(j6, j4, matOfPoint.nativeObj, mat.nativeObj, hitThreshold, size.width, size.height, size2.width, size2.height, matOfPoint2.nativeObj);
    }

    public void detect(Mat img, MatOfPoint foundLocations, MatOfDouble weights, double hitThreshold, Size winStride, Size padding) {
        Size size = winStride;
        Size size2 = padding;
        MatOfPoint matOfPoint = foundLocations;
        MatOfDouble matOfDouble = weights;
        long j = this.nativeObj;
        MatOfPoint matOfPoint2 = matOfPoint;
        MatOfDouble matOfDouble2 = matOfDouble;
        long j2 = j;
        detect_1(j2, img.nativeObj, matOfPoint.nativeObj, matOfDouble.nativeObj, hitThreshold, size.width, size.height, size2.width, size2.height);
    }

    public void detect(Mat img, MatOfPoint foundLocations, MatOfDouble weights, double hitThreshold, Size winStride) {
        Size size = winStride;
        MatOfPoint matOfPoint = foundLocations;
        MatOfDouble matOfDouble = weights;
        MatOfPoint matOfPoint2 = matOfPoint;
        MatOfDouble matOfDouble2 = matOfDouble;
        detect_2(this.nativeObj, img.nativeObj, matOfPoint.nativeObj, matOfDouble.nativeObj, hitThreshold, size.width, size.height);
    }

    public void detect(Mat img, MatOfPoint foundLocations, MatOfDouble weights, double hitThreshold) {
        detect_3(this.nativeObj, img.nativeObj, foundLocations.nativeObj, weights.nativeObj, hitThreshold);
    }

    public void detect(Mat img, MatOfPoint foundLocations, MatOfDouble weights) {
        detect_4(this.nativeObj, img.nativeObj, foundLocations.nativeObj, weights.nativeObj);
    }

    public void detectMultiScale(Mat img, MatOfRect foundLocations, MatOfDouble foundWeights, double hitThreshold, Size winStride, Size padding, double scale, double finalThreshold, boolean useMeanshiftGrouping) {
        Size size = winStride;
        Size size2 = padding;
        MatOfRect matOfRect = foundLocations;
        MatOfDouble matOfDouble = foundWeights;
        long j = this.nativeObj;
        MatOfRect matOfRect2 = matOfRect;
        MatOfDouble matOfDouble2 = matOfDouble;
        long j2 = j;
        detectMultiScale_0(j2, img.nativeObj, matOfRect.nativeObj, matOfDouble.nativeObj, hitThreshold, size.width, size.height, size2.width, size2.height, scale, finalThreshold, useMeanshiftGrouping);
    }

    public void detectMultiScale(Mat img, MatOfRect foundLocations, MatOfDouble foundWeights, double hitThreshold, Size winStride, Size padding, double scale, double finalThreshold) {
        Size size = winStride;
        Size size2 = padding;
        MatOfRect matOfRect = foundLocations;
        MatOfDouble matOfDouble = foundWeights;
        long j = this.nativeObj;
        MatOfRect matOfRect2 = matOfRect;
        MatOfDouble matOfDouble2 = matOfDouble;
        long j2 = j;
        detectMultiScale_1(j2, img.nativeObj, matOfRect.nativeObj, matOfDouble.nativeObj, hitThreshold, size.width, size.height, size2.width, size2.height, scale, finalThreshold);
    }

    public void detectMultiScale(Mat img, MatOfRect foundLocations, MatOfDouble foundWeights, double hitThreshold, Size winStride, Size padding, double scale) {
        Size size = winStride;
        Size size2 = padding;
        MatOfRect matOfRect = foundLocations;
        MatOfDouble matOfDouble = foundWeights;
        long j = this.nativeObj;
        MatOfRect matOfRect2 = matOfRect;
        MatOfDouble matOfDouble2 = matOfDouble;
        long j2 = j;
        detectMultiScale_2(j2, img.nativeObj, matOfRect.nativeObj, matOfDouble.nativeObj, hitThreshold, size.width, size.height, size2.width, size2.height, scale);
    }

    public void detectMultiScale(Mat img, MatOfRect foundLocations, MatOfDouble foundWeights, double hitThreshold, Size winStride, Size padding) {
        Size size = winStride;
        Size size2 = padding;
        MatOfRect matOfRect = foundLocations;
        MatOfDouble matOfDouble = foundWeights;
        long j = this.nativeObj;
        MatOfRect matOfRect2 = matOfRect;
        MatOfDouble matOfDouble2 = matOfDouble;
        long j2 = j;
        detectMultiScale_3(j2, img.nativeObj, matOfRect.nativeObj, matOfDouble.nativeObj, hitThreshold, size.width, size.height, size2.width, size2.height);
    }

    public void detectMultiScale(Mat img, MatOfRect foundLocations, MatOfDouble foundWeights, double hitThreshold, Size winStride) {
        Size size = winStride;
        MatOfRect matOfRect = foundLocations;
        MatOfDouble matOfDouble = foundWeights;
        MatOfRect matOfRect2 = matOfRect;
        MatOfDouble matOfDouble2 = matOfDouble;
        detectMultiScale_4(this.nativeObj, img.nativeObj, matOfRect.nativeObj, matOfDouble.nativeObj, hitThreshold, size.width, size.height);
    }

    public void detectMultiScale(Mat img, MatOfRect foundLocations, MatOfDouble foundWeights, double hitThreshold) {
        detectMultiScale_5(this.nativeObj, img.nativeObj, foundLocations.nativeObj, foundWeights.nativeObj, hitThreshold);
    }

    public void detectMultiScale(Mat img, MatOfRect foundLocations, MatOfDouble foundWeights) {
        detectMultiScale_6(this.nativeObj, img.nativeObj, foundLocations.nativeObj, foundWeights.nativeObj);
    }

    public void save(String filename, String objname) {
        save_0(this.nativeObj, filename, objname);
    }

    public void save(String filename) {
        save_1(this.nativeObj, filename);
    }

    public void setSVMDetector(Mat svmdetector) {
        setSVMDetector_0(this.nativeObj, svmdetector.nativeObj);
    }

    public Size get_winSize() {
        return new Size(get_winSize_0(this.nativeObj));
    }

    public Size get_blockSize() {
        return new Size(get_blockSize_0(this.nativeObj));
    }

    public Size get_blockStride() {
        return new Size(get_blockStride_0(this.nativeObj));
    }

    public Size get_cellSize() {
        return new Size(get_cellSize_0(this.nativeObj));
    }

    public int get_nbins() {
        return get_nbins_0(this.nativeObj);
    }

    public int get_derivAperture() {
        return get_derivAperture_0(this.nativeObj);
    }

    public double get_winSigma() {
        return get_winSigma_0(this.nativeObj);
    }

    public int get_histogramNormType() {
        return get_histogramNormType_0(this.nativeObj);
    }

    public double get_L2HysThreshold() {
        return get_L2HysThreshold_0(this.nativeObj);
    }

    public boolean get_gammaCorrection() {
        return get_gammaCorrection_0(this.nativeObj);
    }

    public MatOfFloat get_svmDetector() {
        return MatOfFloat.fromNativeAddr(get_svmDetector_0(this.nativeObj));
    }

    public int get_nlevels() {
        return get_nlevels_0(this.nativeObj);
    }

    public boolean get_signedGradient() {
        return get_signedGradient_0(this.nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
