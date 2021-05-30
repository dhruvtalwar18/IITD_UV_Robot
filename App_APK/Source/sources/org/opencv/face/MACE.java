package org.opencv.face;

import java.util.List;
import org.opencv.core.Algorithm;
import org.opencv.core.Mat;
import org.opencv.utils.Converters;

public class MACE extends Algorithm {
    private static native long create_0(int i);

    private static native long create_1();

    private static native void delete(long j);

    private static native long load_0(String str, String str2);

    private static native long load_1(String str);

    private static native void salt_0(long j, String str);

    private static native boolean same_0(long j, long j2);

    private static native void train_0(long j, long j2);

    protected MACE(long addr) {
        super(addr);
    }

    public static MACE __fromPtr__(long addr) {
        return new MACE(addr);
    }

    public static MACE create(int IMGSIZE) {
        return __fromPtr__(create_0(IMGSIZE));
    }

    public static MACE create() {
        return __fromPtr__(create_1());
    }

    public static MACE load(String filename, String objname) {
        return __fromPtr__(load_0(filename, objname));
    }

    public static MACE load(String filename) {
        return __fromPtr__(load_1(filename));
    }

    public boolean same(Mat query) {
        return same_0(this.nativeObj, query.nativeObj);
    }

    public void salt(String passphrase) {
        salt_0(this.nativeObj, passphrase);
    }

    public void train(List<Mat> images) {
        train_0(this.nativeObj, Converters.vector_Mat_to_Mat(images).nativeObj);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
