package org.ros.rosjava_geometry;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import java.util.List;
import org.bytedeco.javacpp.opencv_stitching;

public class Quaternion {
    private final double w;
    private final double x;
    private final double y;
    private final double z;

    public static Quaternion fromAxisAngle(Vector3 axis, double angle) {
        Vector3 normalized = axis.normalize();
        double sin = Math.sin(angle / 2.0d);
        return new Quaternion(normalized.getX() * sin, normalized.getY() * sin, normalized.getZ() * sin, Math.cos(angle / 2.0d));
    }

    public static Quaternion fromQuaternionMessage(geometry_msgs.Quaternion message) {
        return new Quaternion(message.getX(), message.getY(), message.getZ(), message.getW());
    }

    public static Quaternion rotationBetweenVectors(Vector3 vector1, Vector3 vector2) {
        boolean z2 = false;
        Preconditions.checkArgument(vector1.getMagnitude() > opencv_stitching.Stitcher.ORIG_RESOL, "Cannot calculate rotation between zero-length vectors.");
        if (vector2.getMagnitude() > opencv_stitching.Stitcher.ORIG_RESOL) {
            z2 = true;
        }
        Preconditions.checkArgument(z2, "Cannot calculate rotation between zero-length vectors.");
        if (vector1.normalize().equals(vector2.normalize())) {
            return identity();
        }
        return fromAxisAngle(new Vector3((vector1.getY() * vector2.getZ()) - (vector1.getZ() * vector2.getY()), (vector1.getZ() * vector2.getX()) - (vector1.getX() * vector2.getZ()), (vector1.getX() * vector2.getY()) - (vector1.getY() * vector2.getX())), Math.acos(vector1.dotProduct(vector2) / (vector1.getMagnitude() * vector2.getMagnitude())));
    }

    public static Quaternion identity() {
        return new Quaternion(opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL, 1.0d);
    }

    public Quaternion(double x2, double y2, double z2, double w2) {
        this.x = x2;
        this.y = y2;
        this.z = z2;
        this.w = w2;
    }

    public Quaternion scale(double factor) {
        return new Quaternion(this.x * factor, this.y * factor, this.z * factor, this.w * factor);
    }

    public Quaternion conjugate() {
        return new Quaternion(-this.x, -this.y, -this.z, this.w);
    }

    public Quaternion invert() {
        double mm = getMagnitudeSquared();
        Preconditions.checkState(mm != opencv_stitching.Stitcher.ORIG_RESOL);
        return conjugate().scale(1.0d / mm);
    }

    public Quaternion normalize() {
        return scale(1.0d / getMagnitude());
    }

    public Quaternion multiply(Quaternion other) {
        return new Quaternion((((this.w * other.x) + (this.x * other.w)) + (this.y * other.z)) - (this.z * other.y), (((this.w * other.y) + (this.y * other.w)) + (this.z * other.x)) - (this.x * other.z), (((this.w * other.z) + (this.z * other.w)) + (this.x * other.y)) - (this.y * other.x), (((this.w * other.w) - (this.x * other.x)) - (this.y * other.y)) - (this.z * other.z));
    }

    public Vector3 rotateAndScaleVector(Vector3 vector) {
        Quaternion rotatedQuaternion = multiply(new Quaternion(vector.getX(), vector.getY(), vector.getZ(), opencv_stitching.Stitcher.ORIG_RESOL).multiply(conjugate()));
        return new Vector3(rotatedQuaternion.getX(), rotatedQuaternion.getY(), rotatedQuaternion.getZ());
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

    public double getW() {
        return this.w;
    }

    public double getMagnitudeSquared() {
        return (this.x * this.x) + (this.y * this.y) + (this.z * this.z) + (this.w * this.w);
    }

    public double getMagnitude() {
        return Math.sqrt(getMagnitudeSquared());
    }

    public boolean isAlmostNeutral(double epsilon) {
        return Math.abs((((1.0d - (this.x * this.x)) - (this.y * this.y)) - (this.z * this.z)) - (this.w * this.w)) < epsilon;
    }

    public geometry_msgs.Quaternion toQuaternionMessage(geometry_msgs.Quaternion result) {
        result.setX(this.x);
        result.setY(this.y);
        result.setZ(this.z);
        result.setW(this.w);
        return result;
    }

    public boolean almostEquals(Quaternion other, double epsilon) {
        List<Double> epsilons = Lists.newArrayList();
        epsilons.add(Double.valueOf(this.x - other.x));
        epsilons.add(Double.valueOf(this.y - other.y));
        epsilons.add(Double.valueOf(this.z - other.z));
        epsilons.add(Double.valueOf(this.w - other.w));
        for (Double doubleValue : epsilons) {
            if (Math.abs(doubleValue.doubleValue()) > epsilon) {
                return false;
            }
        }
        return true;
    }

    public String toString() {
        return String.format("Quaternion<x: %.4f, y: %.4f, z: %.4f, w: %.4f>", new Object[]{Double.valueOf(this.x), Double.valueOf(this.y), Double.valueOf(this.z), Double.valueOf(this.w)});
    }

    public int hashCode() {
        double d = this.w;
        double z2 = opencv_stitching.Stitcher.ORIG_RESOL;
        double w2 = d == opencv_stitching.Stitcher.ORIG_RESOL ? 0.0d : this.w;
        double x2 = this.x == opencv_stitching.Stitcher.ORIG_RESOL ? 0.0d : this.x;
        double y2 = this.y == opencv_stitching.Stitcher.ORIG_RESOL ? 0.0d : this.y;
        if (this.z != opencv_stitching.Stitcher.ORIG_RESOL) {
            z2 = this.z;
        }
        long temp = Double.doubleToLongBits(w2);
        double d2 = w2;
        int result = (1 * 31) + ((int) (temp ^ (temp >>> 32)));
        long temp2 = Double.doubleToLongBits(x2);
        long temp3 = Double.doubleToLongBits(y2);
        long temp4 = Double.doubleToLongBits(z2);
        return (((((result * 31) + ((int) ((temp2 >>> 32) ^ temp2))) * 31) + ((int) ((temp3 >>> 32) ^ temp3))) * 31) + ((int) ((temp4 >>> 32) ^ temp4));
    }

    public boolean equals(Object obj) {
        Object obj2 = obj;
        if (this == obj2) {
            return true;
        }
        if (obj2 == null || getClass() != obj.getClass()) {
            return false;
        }
        Quaternion other = (Quaternion) obj2;
        double d = this.w;
        double otherZ = opencv_stitching.Stitcher.ORIG_RESOL;
        double w2 = d == opencv_stitching.Stitcher.ORIG_RESOL ? 0.0d : this.w;
        double x2 = this.x == opencv_stitching.Stitcher.ORIG_RESOL ? 0.0d : this.x;
        double y2 = this.y == opencv_stitching.Stitcher.ORIG_RESOL ? 0.0d : this.y;
        double z2 = this.z == opencv_stitching.Stitcher.ORIG_RESOL ? 0.0d : this.z;
        double otherW = other.w == opencv_stitching.Stitcher.ORIG_RESOL ? 0.0d : other.w;
        double otherX = other.x == opencv_stitching.Stitcher.ORIG_RESOL ? 0.0d : other.x;
        double z3 = z2;
        double otherY = other.y == opencv_stitching.Stitcher.ORIG_RESOL ? 0.0d : other.y;
        if (other.z != opencv_stitching.Stitcher.ORIG_RESOL) {
            otherZ = other.z;
        }
        if (Double.doubleToLongBits(w2) == Double.doubleToLongBits(otherW) && Double.doubleToLongBits(x2) == Double.doubleToLongBits(otherX) && Double.doubleToLongBits(y2) == Double.doubleToLongBits(otherY) && Double.doubleToLongBits(z3) == Double.doubleToLongBits(otherZ)) {
            return true;
        }
        return false;
    }
}
