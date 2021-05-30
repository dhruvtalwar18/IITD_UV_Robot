package org.opencv.photo;

import java.util.List;
import org.opencv.core.Mat;
import org.opencv.utils.Converters;

public class MergeMertens extends MergeExposures {
    private static native void delete(long j);

    private static native float getContrastWeight_0(long j);

    private static native float getExposureWeight_0(long j);

    private static native float getSaturationWeight_0(long j);

    private static native void process_0(long j, long j2, long j3, long j4, long j5);

    private static native void process_1(long j, long j2, long j3);

    private static native void setContrastWeight_0(long j, float f);

    private static native void setExposureWeight_0(long j, float f);

    private static native void setSaturationWeight_0(long j, float f);

    protected MergeMertens(long addr) {
        super(addr);
    }

    public static MergeMertens __fromPtr__(long addr) {
        return new MergeMertens(addr);
    }

    public float getContrastWeight() {
        return getContrastWeight_0(this.nativeObj);
    }

    public float getExposureWeight() {
        return getExposureWeight_0(this.nativeObj);
    }

    public float getSaturationWeight() {
        return getSaturationWeight_0(this.nativeObj);
    }

    public void process(List<Mat> src, Mat dst, Mat times, Mat response) {
        process_0(this.nativeObj, Converters.vector_Mat_to_Mat(src).nativeObj, dst.nativeObj, times.nativeObj, response.nativeObj);
    }

    public void process(List<Mat> src, Mat dst) {
        process_1(this.nativeObj, Converters.vector_Mat_to_Mat(src).nativeObj, dst.nativeObj);
    }

    public void setContrastWeight(float contrast_weiht) {
        setContrastWeight_0(this.nativeObj, contrast_weiht);
    }

    public void setExposureWeight(float exposure_weight) {
        setExposureWeight_0(this.nativeObj, exposure_weight);
    }

    public void setSaturationWeight(float saturation_weight) {
        setSaturationWeight_0(this.nativeObj, saturation_weight);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
