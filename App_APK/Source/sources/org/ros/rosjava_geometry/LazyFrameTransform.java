package org.ros.rosjava_geometry;

import com.google.common.annotations.VisibleForTesting;
import geometry_msgs.TransformStamped;

public class LazyFrameTransform {
    private FrameTransform frameTransform;
    private final TransformStamped message;
    private final Object mutex;

    public LazyFrameTransform(TransformStamped message2) {
        this.mutex = new Object();
        this.message = message2;
    }

    @VisibleForTesting
    LazyFrameTransform(FrameTransform frameTransform2) {
        this.mutex = new Object();
        this.message = null;
        this.frameTransform = frameTransform2;
    }

    public FrameTransform get() {
        synchronized (this.mutex) {
            if (this.frameTransform != null) {
                FrameTransform frameTransform2 = this.frameTransform;
                return frameTransform2;
            }
            this.frameTransform = FrameTransform.fromTransformStampedMessage(this.message);
            return this.frameTransform;
        }
    }
}
