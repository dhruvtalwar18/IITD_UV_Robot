package org.opencv.structured_light;

import java.util.List;
import org.opencv.core.Algorithm;
import org.opencv.core.Mat;
import org.opencv.utils.Converters;

public class StructuredLightPattern extends Algorithm {
    private static native void delete(long j);

    private static native boolean generate_0(long j, long j2);

    protected StructuredLightPattern(long addr) {
        super(addr);
    }

    public static StructuredLightPattern __fromPtr__(long addr) {
        return new StructuredLightPattern(addr);
    }

    public boolean generate(List<Mat> patternImages) {
        Mat patternImages_mat = new Mat();
        boolean retVal = generate_0(this.nativeObj, patternImages_mat.nativeObj);
        Converters.Mat_to_vector_Mat(patternImages_mat, patternImages);
        patternImages_mat.release();
        return retVal;
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
