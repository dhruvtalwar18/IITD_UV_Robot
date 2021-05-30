package org.opencv.ml;

import org.opencv.core.Mat;
import org.opencv.core.TermCriteria;

public class RTrees extends DTrees {
    private static native long create_0();

    private static native void delete(long j);

    private static native int getActiveVarCount_0(long j);

    private static native boolean getCalculateVarImportance_0(long j);

    private static native double[] getTermCriteria_0(long j);

    private static native long getVarImportance_0(long j);

    private static native void getVotes_0(long j, long j2, long j3, int i);

    private static native long load_0(String str, String str2);

    private static native long load_1(String str);

    private static native void setActiveVarCount_0(long j, int i);

    private static native void setCalculateVarImportance_0(long j, boolean z);

    private static native void setTermCriteria_0(long j, int i, int i2, double d);

    protected RTrees(long addr) {
        super(addr);
    }

    public static RTrees __fromPtr__(long addr) {
        return new RTrees(addr);
    }

    public Mat getVarImportance() {
        return new Mat(getVarImportance_0(this.nativeObj));
    }

    public static RTrees create() {
        return __fromPtr__(create_0());
    }

    public static RTrees load(String filepath, String nodeName) {
        return __fromPtr__(load_0(filepath, nodeName));
    }

    public static RTrees load(String filepath) {
        return __fromPtr__(load_1(filepath));
    }

    public TermCriteria getTermCriteria() {
        return new TermCriteria(getTermCriteria_0(this.nativeObj));
    }

    public boolean getCalculateVarImportance() {
        return getCalculateVarImportance_0(this.nativeObj);
    }

    public int getActiveVarCount() {
        return getActiveVarCount_0(this.nativeObj);
    }

    public void getVotes(Mat samples, Mat results, int flags) {
        getVotes_0(this.nativeObj, samples.nativeObj, results.nativeObj, flags);
    }

    public void setActiveVarCount(int val) {
        setActiveVarCount_0(this.nativeObj, val);
    }

    public void setCalculateVarImportance(boolean val) {
        setCalculateVarImportance_0(this.nativeObj, val);
    }

    public void setTermCriteria(TermCriteria val) {
        setTermCriteria_0(this.nativeObj, val.type, val.maxCount, val.epsilon);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
