package org.ros.android.view.visualization.layer;

import android.view.GestureDetector;
import android.view.MotionEvent;
import com.google.common.base.Preconditions;
import geometry_msgs.PoseStamped;
import javax.microedition.khronos.opengles.GL10;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.android.view.visualization.shape.PixelSpacePoseShape;
import org.ros.android.view.visualization.shape.Shape;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

public class PosePublisherLayer extends DefaultLayer {
    private ConnectedNode connectedNode;
    /* access modifiers changed from: private */
    public GestureDetector gestureDetector;
    /* access modifiers changed from: private */
    public Transform pose;
    private Publisher<PoseStamped> posePublisher;
    /* access modifiers changed from: private */
    public Shape shape;
    private GraphName topic;
    /* access modifiers changed from: private */
    public boolean visible;

    public PosePublisherLayer(String topic2) {
        this(GraphName.of(topic2));
    }

    public PosePublisherLayer(GraphName topic2) {
        this.topic = topic2;
        this.visible = false;
    }

    public void draw(VisualizationView view, GL10 gl) {
        if (this.visible) {
            Preconditions.checkNotNull(this.pose);
            this.shape.draw(view, gl);
        }
    }

    private double angle(double x1, double y1, double x2, double y2) {
        return Math.atan2(y1 - y2, x1 - x2);
    }

    public boolean onTouchEvent(VisualizationView view, MotionEvent event) {
        if (this.visible) {
            Preconditions.checkNotNull(this.pose);
            if (event.getAction() == 2) {
                Vector3 poseVector = this.pose.apply(Vector3.zero());
                Vector3 pointerVector = view.getCamera().toCameraFrame((int) event.getX(), (int) event.getY());
                this.pose = Transform.translation(poseVector).multiply(Transform.zRotation(angle(pointerVector.getX(), pointerVector.getY(), poseVector.getX(), poseVector.getY())));
                this.shape.setTransform(this.pose);
                return true;
            } else if (event.getAction() == 1) {
                this.posePublisher.publish(this.pose.toPoseStampedMessage(view.getCamera().getFrame(), this.connectedNode.getCurrentTime(), this.posePublisher.newMessage()));
                this.visible = false;
                return true;
            }
        }
        this.gestureDetector.onTouchEvent(event);
        return false;
    }

    public void onStart(final VisualizationView view, ConnectedNode connectedNode2) {
        this.connectedNode = connectedNode2;
        this.shape = new PixelSpacePoseShape();
        this.posePublisher = connectedNode2.newPublisher(this.topic, PoseStamped._TYPE);
        view.post(new Runnable() {
            public void run() {
                GestureDetector unused = PosePublisherLayer.this.gestureDetector = new GestureDetector(view.getContext(), new GestureDetector.SimpleOnGestureListener() {
                    public void onLongPress(MotionEvent e) {
                        Transform unused = PosePublisherLayer.this.pose = Transform.translation(view.getCamera().toCameraFrame((int) e.getX(), (int) e.getY()));
                        PosePublisherLayer.this.shape.setTransform(PosePublisherLayer.this.pose);
                        boolean unused2 = PosePublisherLayer.this.visible = true;
                    }
                });
            }
        });
    }

    public void onShutdown(VisualizationView view, Node node) {
        this.posePublisher.shutdown();
    }
}
