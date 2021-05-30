package org.opencv.core;

import org.bytedeco.javacpp.opencv_stitching;

public class Size {
    public double height;
    public double width;

    public Size(double width2, double height2) {
        this.width = width2;
        this.height = height2;
    }

    public Size() {
        this(opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL);
    }

    public Size(Point p) {
        this.width = p.x;
        this.height = p.y;
    }

    public Size(double[] vals) {
        set(vals);
    }

    public void set(double[] vals) {
        double d = opencv_stitching.Stitcher.ORIG_RESOL;
        if (vals != null) {
            this.width = vals.length > 0 ? vals[0] : 0.0d;
            if (vals.length > 1) {
                d = vals[1];
            }
            this.height = d;
            return;
        }
        this.width = opencv_stitching.Stitcher.ORIG_RESOL;
        this.height = opencv_stitching.Stitcher.ORIG_RESOL;
    }

    public double area() {
        return this.width * this.height;
    }

    public boolean empty() {
        return this.width <= opencv_stitching.Stitcher.ORIG_RESOL || this.height <= opencv_stitching.Stitcher.ORIG_RESOL;
    }

    public Size clone() {
        return new Size(this.width, this.height);
    }

    public int hashCode() {
        long temp = Double.doubleToLongBits(this.height);
        int result = (1 * 31) + ((int) ((temp >>> 32) ^ temp));
        long temp2 = Double.doubleToLongBits(this.width);
        return (result * 31) + ((int) ((temp2 >>> 32) ^ temp2));
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof Size)) {
            return false;
        }
        Size it = (Size) obj;
        if (this.width == it.width && this.height == it.height) {
            return true;
        }
        return false;
    }

    public String toString() {
        return ((int) this.width) + "x" + ((int) this.height);
    }
}
