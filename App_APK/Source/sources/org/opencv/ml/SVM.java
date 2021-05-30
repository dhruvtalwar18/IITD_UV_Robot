package org.opencv.ml;

import org.opencv.core.Mat;
import org.opencv.core.TermCriteria;

public class SVM extends StatModel {
    public static final int C = 0;
    public static final int CHI2 = 4;
    public static final int COEF = 4;
    public static final int CUSTOM = -1;
    public static final int C_SVC = 100;
    public static final int DEGREE = 5;
    public static final int EPS_SVR = 103;
    public static final int GAMMA = 1;
    public static final int INTER = 5;
    public static final int LINEAR = 0;
    public static final int NU = 3;
    public static final int NU_SVC = 101;
    public static final int NU_SVR = 104;
    public static final int ONE_CLASS = 102;
    public static final int P = 2;
    public static final int POLY = 1;
    public static final int RBF = 2;
    public static final int SIGMOID = 3;

    private static native long create_0();

    private static native void delete(long j);

    private static native double getC_0(long j);

    private static native long getClassWeights_0(long j);

    private static native double getCoef0_0(long j);

    private static native double getDecisionFunction_0(long j, int i, long j2, long j3);

    private static native long getDefaultGridPtr_0(int i);

    private static native double getDegree_0(long j);

    private static native double getGamma_0(long j);

    private static native int getKernelType_0(long j);

    private static native double getNu_0(long j);

    private static native double getP_0(long j);

    private static native long getSupportVectors_0(long j);

    private static native double[] getTermCriteria_0(long j);

    private static native int getType_0(long j);

    private static native long getUncompressedSupportVectors_0(long j);

    private static native long load_0(String str);

    private static native void setC_0(long j, double d);

    private static native void setClassWeights_0(long j, long j2);

    private static native void setCoef0_0(long j, double d);

    private static native void setDegree_0(long j, double d);

    private static native void setGamma_0(long j, double d);

    private static native void setKernel_0(long j, int i);

    private static native void setNu_0(long j, double d);

    private static native void setP_0(long j, double d);

    private static native void setTermCriteria_0(long j, int i, int i2, double d);

    private static native void setType_0(long j, int i);

    private static native boolean trainAuto_0(long j, long j2, int i, long j3, int i2, long j4, long j5, long j6, long j7, long j8, long j9, boolean z);

    private static native boolean trainAuto_1(long j, long j2, int i, long j3, int i2, long j4, long j5, long j6, long j7, long j8, long j9);

    private static native boolean trainAuto_2(long j, long j2, int i, long j3, int i2, long j4, long j5, long j6, long j7, long j8);

    private static native boolean trainAuto_3(long j, long j2, int i, long j3, int i2, long j4, long j5, long j6, long j7);

    private static native boolean trainAuto_4(long j, long j2, int i, long j3, int i2, long j4, long j5, long j6);

    private static native boolean trainAuto_5(long j, long j2, int i, long j3, int i2, long j4, long j5);

    private static native boolean trainAuto_6(long j, long j2, int i, long j3, int i2, long j4);

    private static native boolean trainAuto_7(long j, long j2, int i, long j3, int i2);

    private static native boolean trainAuto_8(long j, long j2, int i, long j3);

    protected SVM(long addr) {
        super(addr);
    }

    public static SVM __fromPtr__(long addr) {
        return new SVM(addr);
    }

    public Mat getClassWeights() {
        return new Mat(getClassWeights_0(this.nativeObj));
    }

    public Mat getSupportVectors() {
        return new Mat(getSupportVectors_0(this.nativeObj));
    }

    public Mat getUncompressedSupportVectors() {
        return new Mat(getUncompressedSupportVectors_0(this.nativeObj));
    }

    public static ParamGrid getDefaultGridPtr(int param_id) {
        return ParamGrid.__fromPtr__(getDefaultGridPtr_0(param_id));
    }

    public static SVM create() {
        return __fromPtr__(create_0());
    }

    public static SVM load(String filepath) {
        return __fromPtr__(load_0(filepath));
    }

    public TermCriteria getTermCriteria() {
        return new TermCriteria(getTermCriteria_0(this.nativeObj));
    }

    public boolean trainAuto(Mat samples, int layout, Mat responses, int kFold, ParamGrid Cgrid, ParamGrid gammaGrid, ParamGrid pGrid, ParamGrid nuGrid, ParamGrid coeffGrid, ParamGrid degreeGrid, boolean balanced) {
        return trainAuto_0(this.nativeObj, samples.nativeObj, layout, responses.nativeObj, kFold, Cgrid.getNativeObjAddr(), gammaGrid.getNativeObjAddr(), pGrid.getNativeObjAddr(), nuGrid.getNativeObjAddr(), coeffGrid.getNativeObjAddr(), degreeGrid.getNativeObjAddr(), balanced);
    }

    public boolean trainAuto(Mat samples, int layout, Mat responses, int kFold, ParamGrid Cgrid, ParamGrid gammaGrid, ParamGrid pGrid, ParamGrid nuGrid, ParamGrid coeffGrid, ParamGrid degreeGrid) {
        return trainAuto_1(this.nativeObj, samples.nativeObj, layout, responses.nativeObj, kFold, Cgrid.getNativeObjAddr(), gammaGrid.getNativeObjAddr(), pGrid.getNativeObjAddr(), nuGrid.getNativeObjAddr(), coeffGrid.getNativeObjAddr(), degreeGrid.getNativeObjAddr());
    }

    public boolean trainAuto(Mat samples, int layout, Mat responses, int kFold, ParamGrid Cgrid, ParamGrid gammaGrid, ParamGrid pGrid, ParamGrid nuGrid, ParamGrid coeffGrid) {
        return trainAuto_2(this.nativeObj, samples.nativeObj, layout, responses.nativeObj, kFold, Cgrid.getNativeObjAddr(), gammaGrid.getNativeObjAddr(), pGrid.getNativeObjAddr(), nuGrid.getNativeObjAddr(), coeffGrid.getNativeObjAddr());
    }

    public boolean trainAuto(Mat samples, int layout, Mat responses, int kFold, ParamGrid Cgrid, ParamGrid gammaGrid, ParamGrid pGrid, ParamGrid nuGrid) {
        return trainAuto_3(this.nativeObj, samples.nativeObj, layout, responses.nativeObj, kFold, Cgrid.getNativeObjAddr(), gammaGrid.getNativeObjAddr(), pGrid.getNativeObjAddr(), nuGrid.getNativeObjAddr());
    }

    public boolean trainAuto(Mat samples, int layout, Mat responses, int kFold, ParamGrid Cgrid, ParamGrid gammaGrid, ParamGrid pGrid) {
        return trainAuto_4(this.nativeObj, samples.nativeObj, layout, responses.nativeObj, kFold, Cgrid.getNativeObjAddr(), gammaGrid.getNativeObjAddr(), pGrid.getNativeObjAddr());
    }

    public boolean trainAuto(Mat samples, int layout, Mat responses, int kFold, ParamGrid Cgrid, ParamGrid gammaGrid) {
        return trainAuto_5(this.nativeObj, samples.nativeObj, layout, responses.nativeObj, kFold, Cgrid.getNativeObjAddr(), gammaGrid.getNativeObjAddr());
    }

    public boolean trainAuto(Mat samples, int layout, Mat responses, int kFold, ParamGrid Cgrid) {
        return trainAuto_6(this.nativeObj, samples.nativeObj, layout, responses.nativeObj, kFold, Cgrid.getNativeObjAddr());
    }

    public boolean trainAuto(Mat samples, int layout, Mat responses, int kFold) {
        return trainAuto_7(this.nativeObj, samples.nativeObj, layout, responses.nativeObj, kFold);
    }

    public boolean trainAuto(Mat samples, int layout, Mat responses) {
        return trainAuto_8(this.nativeObj, samples.nativeObj, layout, responses.nativeObj);
    }

    public double getC() {
        return getC_0(this.nativeObj);
    }

    public double getCoef0() {
        return getCoef0_0(this.nativeObj);
    }

    public double getDecisionFunction(int i, Mat alpha, Mat svidx) {
        return getDecisionFunction_0(this.nativeObj, i, alpha.nativeObj, svidx.nativeObj);
    }

    public double getDegree() {
        return getDegree_0(this.nativeObj);
    }

    public double getGamma() {
        return getGamma_0(this.nativeObj);
    }

    public double getNu() {
        return getNu_0(this.nativeObj);
    }

    public double getP() {
        return getP_0(this.nativeObj);
    }

    public int getKernelType() {
        return getKernelType_0(this.nativeObj);
    }

    public int getType() {
        return getType_0(this.nativeObj);
    }

    public void setC(double val) {
        setC_0(this.nativeObj, val);
    }

    public void setClassWeights(Mat val) {
        setClassWeights_0(this.nativeObj, val.nativeObj);
    }

    public void setCoef0(double val) {
        setCoef0_0(this.nativeObj, val);
    }

    public void setDegree(double val) {
        setDegree_0(this.nativeObj, val);
    }

    public void setGamma(double val) {
        setGamma_0(this.nativeObj, val);
    }

    public void setKernel(int kernelType) {
        setKernel_0(this.nativeObj, kernelType);
    }

    public void setNu(double val) {
        setNu_0(this.nativeObj, val);
    }

    public void setP(double val) {
        setP_0(this.nativeObj, val);
    }

    public void setTermCriteria(TermCriteria val) {
        setTermCriteria_0(this.nativeObj, val.type, val.maxCount, val.epsilon);
    }

    public void setType(int val) {
        setType_0(this.nativeObj, val);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
