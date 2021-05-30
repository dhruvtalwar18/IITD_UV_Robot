package org.opencv.core;

import java.util.Arrays;
import org.bytedeco.javacpp.opencv_stitching;

public class Scalar {
    public double[] val;

    public Scalar(double v0, double v1, double v2, double v3) {
        this.val = new double[]{v0, v1, v2, v3};
    }

    public Scalar(double v0, double v1, double v2) {
        this.val = new double[]{v0, v1, v2, 0.0d};
    }

    public Scalar(double v0, double v1) {
        this.val = new double[]{v0, v1, 0.0d, 0.0d};
    }

    public Scalar(double v0) {
        this.val = new double[]{v0, 0.0d, 0.0d, 0.0d};
    }

    public Scalar(double[] vals) {
        if (vals == null || vals.length != 4) {
            this.val = new double[4];
            set(vals);
            return;
        }
        this.val = (double[]) vals.clone();
    }

    public void set(double[] vals) {
        double d = opencv_stitching.Stitcher.ORIG_RESOL;
        if (vals != null) {
            this.val[0] = vals.length > 0 ? vals[0] : 0.0d;
            this.val[1] = vals.length > 1 ? vals[1] : 0.0d;
            this.val[2] = vals.length > 2 ? vals[2] : 0.0d;
            double[] dArr = this.val;
            if (vals.length > 3) {
                d = vals[3];
            }
            dArr[3] = d;
            return;
        }
        double[] dArr2 = this.val;
        double[] dArr3 = this.val;
        double[] dArr4 = this.val;
        this.val[3] = 0.0d;
        dArr4[2] = 0.0d;
        dArr3[1] = 0.0d;
        dArr2[0] = 0.0d;
    }

    public static Scalar all(double v) {
        return new Scalar(v, v, v, v);
    }

    public Scalar clone() {
        return new Scalar(this.val);
    }

    public Scalar mul(Scalar it, double scale) {
        return new Scalar(this.val[0] * it.val[0] * scale, this.val[1] * it.val[1] * scale, this.val[2] * it.val[2] * scale, this.val[3] * it.val[3] * scale);
    }

    public Scalar mul(Scalar it) {
        return mul(it, 1.0d);
    }

    public Scalar conj() {
        return new Scalar(this.val[0], -this.val[1], -this.val[2], -this.val[3]);
    }

    public boolean isReal() {
        return this.val[1] == opencv_stitching.Stitcher.ORIG_RESOL && this.val[2] == opencv_stitching.Stitcher.ORIG_RESOL && this.val[3] == opencv_stitching.Stitcher.ORIG_RESOL;
    }

    public int hashCode() {
        return (1 * 31) + Arrays.hashCode(this.val);
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if ((obj instanceof Scalar) && Arrays.equals(this.val, ((Scalar) obj).val)) {
            return true;
        }
        return false;
    }

    public String toString() {
        return "[" + this.val[0] + ", " + this.val[1] + ", " + this.val[2] + ", " + this.val[3] + "]";
    }
}
