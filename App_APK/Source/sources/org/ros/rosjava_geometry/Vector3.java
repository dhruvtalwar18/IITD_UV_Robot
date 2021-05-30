package org.ros.rosjava_geometry;

import com.google.common.collect.Lists;
import geometry_msgs.Point;
import java.util.List;
import org.bytedeco.javacpp.opencv_stitching;

public class Vector3 {
    private static final Vector3 X_AXIS = new Vector3(1.0d, opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL);
    private static final Vector3 Y_AXIS = new Vector3(opencv_stitching.Stitcher.ORIG_RESOL, 1.0d, opencv_stitching.Stitcher.ORIG_RESOL);
    private static final Vector3 ZERO = new Vector3(opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL);
    private static final Vector3 Z_AXIS = new Vector3(opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL, 1.0d);
    private final double x;
    private final double y;
    private final double z;

    public static Vector3 fromVector3Message(geometry_msgs.Vector3 message) {
        return new Vector3(message.getX(), message.getY(), message.getZ());
    }

    public static Vector3 fromPointMessage(Point message) {
        return new Vector3(message.getX(), message.getY(), message.getZ());
    }

    public static Vector3 zero() {
        return ZERO;
    }

    public static Vector3 xAxis() {
        return X_AXIS;
    }

    public static Vector3 yAxis() {
        return Y_AXIS;
    }

    public static Vector3 zAxis() {
        return Z_AXIS;
    }

    public Vector3(double x2, double y2, double z2) {
        this.x = x2;
        this.y = y2;
        this.z = z2;
    }

    public Vector3 add(Vector3 other) {
        return new Vector3(this.x + other.x, this.y + other.y, this.z + other.z);
    }

    public Vector3 subtract(Vector3 other) {
        return new Vector3(this.x - other.x, this.y - other.y, this.z - other.z);
    }

    public Vector3 invert() {
        return new Vector3(-this.x, -this.y, -this.z);
    }

    public double dotProduct(Vector3 other) {
        return (this.x * other.x) + (this.y * other.y) + (this.z * other.z);
    }

    public Vector3 normalize() {
        return new Vector3(this.x / getMagnitude(), this.y / getMagnitude(), this.z / getMagnitude());
    }

    public Vector3 scale(double factor) {
        return new Vector3(this.x * factor, this.y * factor, this.z * factor);
    }

    public double getX() {
        return this.x;
    }

    public double getY() {
        return this.y;
    }

    public double getZ() {
        return this.z;
    }

    public double getMagnitudeSquared() {
        return (this.x * this.x) + (this.y * this.y) + (this.z * this.z);
    }

    public double getMagnitude() {
        return Math.sqrt(getMagnitudeSquared());
    }

    public geometry_msgs.Vector3 toVector3Message(geometry_msgs.Vector3 result) {
        result.setX(this.x);
        result.setY(this.y);
        result.setZ(this.z);
        return result;
    }

    public Point toPointMessage(Point result) {
        result.setX(this.x);
        result.setY(this.y);
        result.setZ(this.z);
        return result;
    }

    public boolean almostEquals(Vector3 other, double epsilon) {
        List<Double> epsilons = Lists.newArrayList();
        epsilons.add(Double.valueOf(this.x - other.x));
        epsilons.add(Double.valueOf(this.y - other.y));
        epsilons.add(Double.valueOf(this.z - other.z));
        for (Double doubleValue : epsilons) {
            if (Math.abs(doubleValue.doubleValue()) > epsilon) {
                return false;
            }
        }
        return true;
    }

    public String toString() {
        return String.format("Vector3<x: %.4f, y: %.4f, z: %.4f>", new Object[]{Double.valueOf(this.x), Double.valueOf(this.y), Double.valueOf(this.z)});
    }

    public int hashCode() {
        double d = this.x;
        double z2 = opencv_stitching.Stitcher.ORIG_RESOL;
        double x2 = d == opencv_stitching.Stitcher.ORIG_RESOL ? 0.0d : this.x;
        double y2 = this.y == opencv_stitching.Stitcher.ORIG_RESOL ? 0.0d : this.y;
        if (this.z != opencv_stitching.Stitcher.ORIG_RESOL) {
            z2 = this.z;
        }
        long temp = Double.doubleToLongBits(x2);
        long temp2 = Double.doubleToLongBits(y2);
        long temp3 = Double.doubleToLongBits(z2);
        return (((((1 * 31) + ((int) ((temp >>> 32) ^ temp))) * 31) + ((int) ((temp2 >>> 32) ^ temp2))) * 31) + ((int) ((temp3 >>> 32) ^ temp3));
    }

    public boolean equals(Object obj) {
        Object obj2 = obj;
        if (this == obj2) {
            return true;
        }
        if (obj2 == null || getClass() != obj.getClass()) {
            return false;
        }
        Vector3 other = (Vector3) obj2;
        double d = this.x;
        double d2 = opencv_stitching.Stitcher.ORIG_RESOL;
        double x2 = d == opencv_stitching.Stitcher.ORIG_RESOL ? 0.0d : this.x;
        double y2 = this.y == opencv_stitching.Stitcher.ORIG_RESOL ? 0.0d : this.y;
        double z2 = this.z == opencv_stitching.Stitcher.ORIG_RESOL ? 0.0d : this.z;
        double otherX = other.x == opencv_stitching.Stitcher.ORIG_RESOL ? 0.0d : other.x;
        double otherY = other.y == opencv_stitching.Stitcher.ORIG_RESOL ? 0.0d : other.y;
        if (other.z != opencv_stitching.Stitcher.ORIG_RESOL) {
            d2 = other.z;
        }
        double otherZ = d2;
        if (Double.doubleToLongBits(x2) == Double.doubleToLongBits(otherX) && Double.doubleToLongBits(y2) == Double.doubleToLongBits(otherY) && Double.doubleToLongBits(z2) == Double.doubleToLongBits(otherZ)) {
            return true;
        }
        return false;
    }
}
