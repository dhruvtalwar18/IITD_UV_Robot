package org.ros.android.view.visualization;

import com.google.common.base.Preconditions;
import javax.microedition.khronos.opengles.GL10;
import org.bytedeco.javacpp.opencv_stitching;
import org.ros.math.RosMath;
import org.ros.namespace.GraphName;
import org.ros.rosjava_geometry.FrameTransform;
import org.ros.rosjava_geometry.FrameTransformTree;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

public class XYOrthographicCamera {
    private static final float MAXIMUM_ZOOM_FACTOR = 5.0f;
    private static final float MINIMUM_ZOOM_FACTOR = 0.1f;
    private static final double PIXELS_PER_METER = 100.0d;
    private static final Transform ROS_TO_SCREEN_TRANSFORM = Transform.zRotation(1.5707963267948966d).scale(PIXELS_PER_METER);
    private Transform cameraToRosTransform;
    private GraphName frame;
    private final FrameTransformTree frameTransformTree;
    private final Object mutex = new Object();
    private Viewport viewport;

    public XYOrthographicCamera(FrameTransformTree frameTransformTree2) {
        this.frameTransformTree = frameTransformTree2;
        resetTransform();
    }

    private void resetTransform() {
        this.cameraToRosTransform = Transform.identity();
    }

    public void apply(GL10 gl) {
        synchronized (this.mutex) {
            OpenGlTransform.apply(gl, ROS_TO_SCREEN_TRANSFORM);
            OpenGlTransform.apply(gl, this.cameraToRosTransform);
        }
    }

    public boolean applyFrameTransform(GL10 gl, GraphName frame2) {
        FrameTransform frameTransform;
        Preconditions.checkNotNull(frame2);
        if (this.frame == null || (frameTransform = this.frameTransformTree.transform(frame2, this.frame)) == null) {
            return false;
        }
        OpenGlTransform.apply(gl, frameTransform.getTransform());
        return true;
    }

    public void translate(double deltaX, double deltaY) {
        synchronized (this.mutex) {
            this.cameraToRosTransform = ROS_TO_SCREEN_TRANSFORM.invert().multiply(Transform.translation(deltaX, deltaY, opencv_stitching.Stitcher.ORIG_RESOL)).multiply(getCameraToScreenTransform());
        }
    }

    private Transform getCameraToScreenTransform() {
        return ROS_TO_SCREEN_TRANSFORM.multiply(this.cameraToRosTransform);
    }

    public Transform getScreenTransform(GraphName targetFrame) {
        return this.frameTransformTree.transform(this.frame, targetFrame).getTransform().multiply(getCameraToScreenTransform().invert());
    }

    public void rotate(double focusX, double focusY, double deltaAngle) {
        synchronized (this.mutex) {
            Transform focus = Transform.translation(toCameraFrame((int) focusX, (int) focusY));
            this.cameraToRosTransform = this.cameraToRosTransform.multiply(focus).multiply(Transform.zRotation(deltaAngle)).multiply(focus.invert());
        }
    }

    public void zoom(double focusX, double focusY, double factor) {
        synchronized (this.mutex) {
            Transform focus = Transform.translation(toCameraFrame((int) focusX, (int) focusY));
            double scale = this.cameraToRosTransform.getScale();
            this.cameraToRosTransform = this.cameraToRosTransform.multiply(focus).scale(RosMath.clamp(scale * factor, 0.10000000149011612d, 5.0d) / scale).multiply(focus.invert());
        }
    }

    public double getZoom() {
        return this.cameraToRosTransform.getScale() * PIXELS_PER_METER;
    }

    public Vector3 toCameraFrame(int pixelX, int pixelY) {
        double d = (double) pixelX;
        double width = (double) this.viewport.getWidth();
        Double.isNaN(width);
        Double.isNaN(d);
        double centeredX = d - (width / 2.0d);
        double height = (double) this.viewport.getHeight();
        Double.isNaN(height);
        double d2 = (double) pixelY;
        Double.isNaN(d2);
        return getCameraToScreenTransform().invert().apply(new Vector3(centeredX, (height / 2.0d) - d2, opencv_stitching.Stitcher.ORIG_RESOL));
    }

    public Transform toFrame(int pixelX, int pixelY, GraphName frame2) {
        return this.frameTransformTree.transform(this.frame, frame2).getTransform().multiply(Transform.translation(toCameraFrame(pixelX, pixelY)));
    }

    public GraphName getFrame() {
        return this.frame;
    }

    public void setFrame(GraphName frame2) {
        FrameTransform frameTransform;
        Preconditions.checkNotNull(frame2);
        synchronized (this.mutex) {
            if (!(this.frame == null || this.frame == frame2 || (frameTransform = this.frameTransformTree.transform(frame2, this.frame)) == null)) {
                this.cameraToRosTransform = this.cameraToRosTransform.multiply(frameTransform.getTransform());
            }
            this.frame = frame2;
        }
    }

    public void setFrame(String frame2) {
        setFrame(GraphName.of(frame2));
    }

    public void jumpToFrame(GraphName frame2) {
        synchronized (this.mutex) {
            this.frame = frame2;
            double scale = this.cameraToRosTransform.getScale();
            resetTransform();
            this.cameraToRosTransform = this.cameraToRosTransform.scale(scale / this.cameraToRosTransform.getScale());
        }
    }

    public void jumpToFrame(String frame2) {
        jumpToFrame(GraphName.of(frame2));
    }

    public void setViewport(Viewport viewport2) {
        Preconditions.checkNotNull(viewport2);
        this.viewport = viewport2;
    }

    public Viewport getViewport() {
        Preconditions.checkNotNull(this.viewport);
        return this.viewport;
    }

    public Transform getCameraToRosTransform() {
        return this.cameraToRosTransform;
    }
}
