package org.opencv.core;

import org.bytedeco.javacpp.opencv_stitching;

public class Rect2d {
    public double height;
    public double width;
    public double x;
    public double y;

    public Rect2d(double x2, double y2, double width2, double height2) {
        this.x = x2;
        this.y = y2;
        this.width = width2;
        this.height = height2;
    }

    public Rect2d() {
        this(opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL);
    }

    public Rect2d(Point p1, Point p2) {
        this.x = p1.x < p2.x ? p1.x : p2.x;
        this.y = p1.y < p2.y ? p1.y : p2.y;
        this.width = (p1.x > p2.x ? p1.x : p2.x) - this.x;
        this.height = (p1.y > p2.y ? p1.y : p2.y) - this.y;
    }

    public Rect2d(Point p, Size s) {
        this(p.x, p.y, s.width, s.height);
    }

    public Rect2d(double[] vals) {
        set(vals);
    }

    public void set(double[] vals) {
        double d = opencv_stitching.Stitcher.ORIG_RESOL;
        if (vals != null) {
            this.x = vals.length > 0 ? vals[0] : 0.0d;
            this.y = vals.length > 1 ? vals[1] : 0.0d;
            this.width = vals.length > 2 ? vals[2] : 0.0d;
            if (vals.length > 3) {
                d = vals[3];
            }
            this.height = d;
            return;
        }
        this.x = opencv_stitching.Stitcher.ORIG_RESOL;
        this.y = opencv_stitching.Stitcher.ORIG_RESOL;
        this.width = opencv_stitching.Stitcher.ORIG_RESOL;
        this.height = opencv_stitching.Stitcher.ORIG_RESOL;
    }

    public Rect2d clone() {
        return new Rect2d(this.x, this.y, this.width, this.height);
    }

    public Point tl() {
        return new Point(this.x, this.y);
    }

    public Point br() {
        return new Point(this.x + this.width, this.y + this.height);
    }

    public Size size() {
        return new Size(this.width, this.height);
    }

    public double area() {
        return this.width * this.height;
    }

    public boolean empty() {
        return this.width <= opencv_stitching.Stitcher.ORIG_RESOL || this.height <= opencv_stitching.Stitcher.ORIG_RESOL;
    }

    public boolean contains(Point p) {
        return this.x <= p.x && p.x < this.x + this.width && this.y <= p.y && p.y < this.y + this.height;
    }

    public int hashCode() {
        long temp = Double.doubleToLongBits(this.height);
        long temp2 = Double.doubleToLongBits(this.width);
        int result = (((1 * 31) + ((int) ((temp >>> 32) ^ temp))) * 31) + ((int) ((temp2 >>> 32) ^ temp2));
        long temp3 = Double.doubleToLongBits(this.x);
        int result2 = (result * 31) + ((int) ((temp3 >>> 32) ^ temp3));
        long temp4 = Double.doubleToLongBits(this.y);
        return (result2 * 31) + ((int) ((temp4 >>> 32) ^ temp4));
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof Rect2d)) {
            return false;
        }
        Rect2d it = (Rect2d) obj;
        if (this.x == it.x && this.y == it.y && this.width == it.width && this.height == it.height) {
            return true;
        }
        return false;
    }

    public String toString() {
        return "{" + this.x + ", " + this.y + ", " + this.width + "x" + this.height + "}";
    }
}
