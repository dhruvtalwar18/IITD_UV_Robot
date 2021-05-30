package org.ros.android.view.visualization.layer;

import geometry_msgs.PoseStamped;
import javax.microedition.khronos.opengles.GL10;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.android.view.visualization.shape.GoalShape;
import org.ros.android.view.visualization.shape.Shape;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.rosjava_geometry.FrameTransform;
import org.ros.rosjava_geometry.Transform;

public class PoseSubscriberLayer extends SubscriberLayer<PoseStamped> implements TfLayer {
    /* access modifiers changed from: private */
    public boolean ready;
    /* access modifiers changed from: private */
    public Shape shape;
    /* access modifiers changed from: private */
    public final GraphName targetFrame;

    public PoseSubscriberLayer(String topic) {
        this(GraphName.of(topic));
    }

    public PoseSubscriberLayer(GraphName topic) {
        super(topic, PoseStamped._TYPE);
        this.targetFrame = GraphName.of("map");
        this.ready = false;
    }

    public void draw(VisualizationView view, GL10 gl) {
        if (this.ready) {
            this.shape.draw(view, gl);
        }
    }

    public void onStart(final VisualizationView view, ConnectedNode connectedNode) {
        super.onStart(view, connectedNode);
        this.shape = new GoalShape();
        getSubscriber().addMessageListener(new MessageListener<PoseStamped>() {
            public void onNewMessage(PoseStamped pose) {
                FrameTransform frameTransform = view.getFrameTransformTree().transform(GraphName.of(pose.getHeader().getFrameId()), PoseSubscriberLayer.this.targetFrame);
                if (frameTransform != null) {
                    PoseSubscriberLayer.this.shape.setTransform(frameTransform.getTransform().multiply(Transform.fromPoseMessage(pose.getPose())));
                    boolean unused = PoseSubscriberLayer.this.ready = true;
                }
            }
        });
    }

    public GraphName getFrame() {
        return this.targetFrame;
    }
}
