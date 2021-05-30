package org.opencv.ximgproc;

import org.opencv.core.Algorithm;
import org.opencv.core.Mat;
import org.opencv.core.MatOfRect;

public class EdgeBoxes extends Algorithm {
    private static native void delete(long j);

    private static native float getAlpha_0(long j);

    private static native float getBeta_0(long j);

    private static native void getBoundingBoxes_0(long j, long j2, long j3, long j4);

    private static native float getClusterMinMag_0(long j);

    private static native float getEdgeMergeThr_0(long j);

    private static native float getEdgeMinMag_0(long j);

    private static native float getEta_0(long j);

    private static native float getGamma_0(long j);

    private static native float getKappa_0(long j);

    private static native float getMaxAspectRatio_0(long j);

    private static native int getMaxBoxes_0(long j);

    private static native float getMinBoxArea_0(long j);

    private static native float getMinScore_0(long j);

    private static native void setAlpha_0(long j, float f);

    private static native void setBeta_0(long j, float f);

    private static native void setClusterMinMag_0(long j, float f);

    private static native void setEdgeMergeThr_0(long j, float f);

    private static native void setEdgeMinMag_0(long j, float f);

    private static native void setEta_0(long j, float f);

    private static native void setGamma_0(long j, float f);

    private static native void setKappa_0(long j, float f);

    private static native void setMaxAspectRatio_0(long j, float f);

    private static native void setMaxBoxes_0(long j, int i);

    private static native void setMinBoxArea_0(long j, float f);

    private static native void setMinScore_0(long j, float f);

    protected EdgeBoxes(long addr) {
        super(addr);
    }

    public static EdgeBoxes __fromPtr__(long addr) {
        return new EdgeBoxes(addr);
    }

    public float getAlpha() {
        return getAlpha_0(this.nativeObj);
    }

    public float getBeta() {
        return getBeta_0(this.nativeObj);
    }

    public float getClusterMinMag() {
        return getClusterMinMag_0(this.nativeObj);
    }

    public float getEdgeMergeThr() {
        return getEdgeMergeThr_0(this.nativeObj);
    }

    public float getEdgeMinMag() {
        return getEdgeMinMag_0(this.nativeObj);
    }

    public float getEta() {
        return getEta_0(this.nativeObj);
    }

    public float getGamma() {
        return getGamma_0(this.nativeObj);
    }

    public float getKappa() {
        return getKappa_0(this.nativeObj);
    }

    public float getMaxAspectRatio() {
        return getMaxAspectRatio_0(this.nativeObj);
    }

    public float getMinBoxArea() {
        return getMinBoxArea_0(this.nativeObj);
    }

    public float getMinScore() {
        return getMinScore_0(this.nativeObj);
    }

    public int getMaxBoxes() {
        return getMaxBoxes_0(this.nativeObj);
    }

    public void getBoundingBoxes(Mat edge_map, Mat orientation_map, MatOfRect boxes) {
        getBoundingBoxes_0(this.nativeObj, edge_map.nativeObj, orientation_map.nativeObj, boxes.nativeObj);
    }

    public void setAlpha(float value) {
        setAlpha_0(this.nativeObj, value);
    }

    public void setBeta(float value) {
        setBeta_0(this.nativeObj, value);
    }

    public void setClusterMinMag(float value) {
        setClusterMinMag_0(this.nativeObj, value);
    }

    public void setEdgeMergeThr(float value) {
        setEdgeMergeThr_0(this.nativeObj, value);
    }

    public void setEdgeMinMag(float value) {
        setEdgeMinMag_0(this.nativeObj, value);
    }

    public void setEta(float value) {
        setEta_0(this.nativeObj, value);
    }

    public void setGamma(float value) {
        setGamma_0(this.nativeObj, value);
    }

    public void setKappa(float value) {
        setKappa_0(this.nativeObj, value);
    }

    public void setMaxAspectRatio(float value) {
        setMaxAspectRatio_0(this.nativeObj, value);
    }

    public void setMaxBoxes(int value) {
        setMaxBoxes_0(this.nativeObj, value);
    }

    public void setMinBoxArea(float value) {
        setMinBoxArea_0(this.nativeObj, value);
    }

    public void setMinScore(float value) {
        setMinScore_0(this.nativeObj, value);
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        delete(this.nativeObj);
    }
}
