package org.ros.rosjava_geometry;

import geometry_msgs.Pose;
import geometry_msgs.PoseStamped;
import org.ros.message.Time;
import org.ros.namespace.GraphName;

public class Transform {
    private Quaternion rotationAndScale;
    private Vector3 translation;

    public static Transform fromTransformMessage(geometry_msgs.Transform message) {
        return new Transform(Vector3.fromVector3Message(message.getTranslation()), Quaternion.fromQuaternionMessage(message.getRotation()));
    }

    public static Transform fromPoseMessage(Pose message) {
        return new Transform(Vector3.fromPointMessage(message.getPosition()), Quaternion.fromQuaternionMessage(message.getOrientation()));
    }

    public static Transform identity() {
        return new Transform(Vector3.zero(), Quaternion.identity());
    }

    public static Transform xRotation(double angle) {
        return new Transform(Vector3.zero(), Quaternion.fromAxisAngle(Vector3.xAxis(), angle));
    }

    public static Transform yRotation(double angle) {
        return new Transform(Vector3.zero(), Quaternion.fromAxisAngle(Vector3.yAxis(), angle));
    }

    public static Transform zRotation(double angle) {
        return new Transform(Vector3.zero(), Quaternion.fromAxisAngle(Vector3.zAxis(), angle));
    }

    public static Transform translation(double x, double y, double z) {
        return new Transform(new Vector3(x, y, z), Quaternion.identity());
    }

    public static Transform translation(Vector3 vector) {
        return new Transform(vector, Quaternion.identity());
    }

    public Transform(Vector3 translation2, Quaternion rotation) {
        this.translation = translation2;
        this.rotationAndScale = rotation;
    }

    public Transform multiply(Transform other) {
        return new Transform(apply(other.translation), apply(other.rotationAndScale));
    }

    public Transform invert() {
        Quaternion inverseRotationAndScale = this.rotationAndScale.invert();
        return new Transform(inverseRotationAndScale.rotateAndScaleVector(this.translation.invert()), inverseRotationAndScale);
    }

    public Vector3 apply(Vector3 vector) {
        return this.rotationAndScale.rotateAndScaleVector(vector).add(this.translation);
    }

    public Quaternion apply(Quaternion quaternion) {
        return this.rotationAndScale.multiply(quaternion);
    }

    public Transform scale(double factor) {
        return new Transform(this.translation, this.rotationAndScale.scale(Math.sqrt(factor)));
    }

    public double getScale() {
        return this.rotationAndScale.getMagnitudeSquared();
    }

    public double[] toMatrix() {
        double x = this.rotationAndScale.getX();
        double y = this.rotationAndScale.getY();
        double z = this.rotationAndScale.getZ();
        double w = this.rotationAndScale.getW();
        double mm = this.rotationAndScale.getMagnitudeSquared();
        return new double[]{(mm - ((y * 2.0d) * y)) - ((z * 2.0d) * z), (x * 2.0d * y) + (z * 2.0d * w), ((x * 2.0d) * z) - ((y * 2.0d) * w), 0.0d, ((x * 2.0d) * y) - ((z * 2.0d) * w), (mm - ((x * 2.0d) * x)) - ((z * 2.0d) * z), (y * 2.0d * z) + (x * 2.0d * w), 0.0d, (x * 2.0d * z) + (y * 2.0d * w), ((y * 2.0d) * z) - ((x * 2.0d) * w), (mm - ((x * 2.0d) * x)) - ((2.0d * y) * y), 0.0d, this.translation.getX(), this.translation.getY(), this.translation.getZ(), 1.0d};
    }

    public geometry_msgs.Transform toTransformMessage(geometry_msgs.Transform result) {
        result.setTranslation(this.translation.toVector3Message(result.getTranslation()));
        result.setRotation(this.rotationAndScale.toQuaternionMessage(result.getRotation()));
        return result;
    }

    public Pose toPoseMessage(Pose result) {
        result.setPosition(this.translation.toPointMessage(result.getPosition()));
        result.setOrientation(this.rotationAndScale.toQuaternionMessage(result.getOrientation()));
        return result;
    }

    public PoseStamped toPoseStampedMessage(GraphName frame, Time stamp, PoseStamped result) {
        result.getHeader().setFrameId(frame.toString());
        result.getHeader().setStamp(stamp);
        result.setPose(toPoseMessage(result.getPose()));
        return result;
    }

    public boolean almostEquals(Transform other, double epsilon) {
        return this.translation.almostEquals(other.translation, epsilon) && this.rotationAndScale.almostEquals(other.rotationAndScale, epsilon);
    }

    public Vector3 getTranslation() {
        return this.translation;
    }

    public Quaternion getRotationAndScale() {
        return this.rotationAndScale;
    }

    public String toString() {
        return String.format("Transform<%s, %s>", new Object[]{this.translation, this.rotationAndScale});
    }

    public int hashCode() {
        int i = 0;
        int result = ((1 * 31) + (this.rotationAndScale == null ? 0 : this.rotationAndScale.hashCode())) * 31;
        if (this.translation != null) {
            i = this.translation.hashCode();
        }
        return result + i;
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        Transform other = (Transform) obj;
        if (this.rotationAndScale == null) {
            if (other.rotationAndScale != null) {
                return false;
            }
        } else if (!this.rotationAndScale.equals(other.rotationAndScale)) {
            return false;
        }
        if (this.translation == null) {
            if (other.translation != null) {
                return false;
            }
        } else if (!this.translation.equals(other.translation)) {
            return false;
        }
        return true;
    }
}
