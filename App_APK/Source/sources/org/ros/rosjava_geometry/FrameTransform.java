package org.ros.rosjava_geometry;

import com.google.common.base.Preconditions;
import geometry_msgs.TransformStamped;
import org.ros.message.Time;
import org.ros.namespace.GraphName;

public class FrameTransform {
    private final GraphName source;
    private final GraphName target;
    private final Time time;
    private final Transform transform;

    public static FrameTransform fromTransformStampedMessage(TransformStamped transformStamped) {
        Transform transform2 = Transform.fromTransformMessage(transformStamped.getTransform());
        String target2 = transformStamped.getHeader().getFrameId();
        String source2 = transformStamped.getChildFrameId();
        return new FrameTransform(transform2, GraphName.of(source2), GraphName.of(target2), transformStamped.getHeader().getStamp());
    }

    public FrameTransform(Transform transform2, GraphName source2, GraphName target2, Time time2) {
        Preconditions.checkNotNull(transform2);
        Preconditions.checkNotNull(source2);
        Preconditions.checkNotNull(target2);
        this.transform = transform2;
        this.source = source2.toRelative();
        this.target = target2.toRelative();
        this.time = time2;
    }

    public Transform getTransform() {
        return this.transform;
    }

    public GraphName getSourceFrame() {
        return this.source;
    }

    public GraphName getTargetFrame() {
        return this.target;
    }

    public FrameTransform invert() {
        return new FrameTransform(this.transform.invert(), this.target, this.source, this.time);
    }

    public Time getTime() {
        return this.time;
    }

    public TransformStamped toTransformStampedMessage(TransformStamped result) {
        Preconditions.checkNotNull(this.time);
        result.getHeader().setFrameId(this.target.toString());
        result.getHeader().setStamp(this.time);
        result.setChildFrameId(this.source.toString());
        this.transform.toTransformMessage(result.getTransform());
        return result;
    }

    public String toString() {
        return String.format("FrameTransform<Source: %s, Target: %s, %s, Time: %s>", new Object[]{this.source, this.target, this.transform, this.time});
    }

    public int hashCode() {
        int i = 0;
        int result = ((((((1 * 31) + (this.source == null ? 0 : this.source.hashCode())) * 31) + (this.target == null ? 0 : this.target.hashCode())) * 31) + (this.time == null ? 0 : this.time.hashCode())) * 31;
        if (this.transform != null) {
            i = this.transform.hashCode();
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
        FrameTransform other = (FrameTransform) obj;
        if (this.source == null) {
            if (other.source != null) {
                return false;
            }
        } else if (!this.source.equals(other.source)) {
            return false;
        }
        if (this.target == null) {
            if (other.target != null) {
                return false;
            }
        } else if (!this.target.equals(other.target)) {
            return false;
        }
        if (this.time == null) {
            if (other.time != null) {
                return false;
            }
        } else if (!this.time.equals(other.time)) {
            return false;
        }
        if (this.transform == null) {
            if (other.transform != null) {
                return false;
            }
        } else if (!this.transform.equals(other.transform)) {
            return false;
        }
        return true;
    }
}
