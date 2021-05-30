package org.opencv.face;

import java.util.List;
import org.opencv.core.Algorithm;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfRect;
import org.opencv.utils.Converters;

public class Facemark extends Algorithm {
    private static native void delete(long j);

    private static native boolean fit_0(long j, long j2, long j3, long j4);

    private static native void loadModel_0(long j, String str);

    protected Facemark(long addr) {
        super(addr);
    }

    public static Facemark __fromPtr__(long addr) {
        return new Facemark(addr);
    }

    public boolean fit(Mat image, MatOfRect faces, List<MatOfPoint2f> landmarks) {
        Mat landmarks_mat = new Mat();
        boolean retVal = fit_0(this.nativeObj, image.nativeObj, faces.nativeObj, landmarks_mat.nativeObj);
        Converters.Mat_to_vector_vector_Point2f(landmarks_mat, landmarks);
        landmarks_mat.release();
        return retVal;
    }

    public void loadModel(String model) {
        loadModel_0(this.nativeObj, model);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
