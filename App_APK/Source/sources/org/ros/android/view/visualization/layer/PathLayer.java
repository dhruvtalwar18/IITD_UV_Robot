package org.ros.android.view.visualization.layer;

import geometry_msgs.PoseStamped;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import javax.microedition.khronos.opengles.GL10;
import nav_msgs.Path;
import org.ros.android.view.visualization.Color;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;

public class PathLayer extends SubscriberLayer<Path> implements TfLayer {
    private static final Color COLOR = Color.fromHexAndAlpha("03dfc9", 0.3f);
    private static final float LINE_WIDTH = 4.0f;
    private GraphName frame;
    private int numPoints;
    /* access modifiers changed from: private */
    public boolean ready;
    private FloatBuffer vertexBuffer;

    public PathLayer(String topic) {
        this(GraphName.of(topic));
    }

    public PathLayer(GraphName topic) {
        super(topic, Path._TYPE);
        this.ready = false;
        this.numPoints = 0;
    }

    public void draw(VisualizationView view, GL10 gl) {
        if (this.ready) {
            gl.glEnableClientState(32884);
            gl.glVertexPointer(3, 5126, 0, this.vertexBuffer);
            COLOR.apply(gl);
            gl.glLineWidth(LINE_WIDTH);
            gl.glDrawArrays(3, 0, this.numPoints);
            gl.glDisableClientState(32884);
        }
    }

    public void onStart(VisualizationView view, ConnectedNode connectedNode) {
        super.onStart(view, connectedNode);
        getSubscriber().addMessageListener(new MessageListener<Path>() {
            public void onNewMessage(Path path) {
                PathLayer.this.updateVertexBuffer(path);
                boolean unused = PathLayer.this.ready = true;
            }
        });
    }

    /* access modifiers changed from: private */
    public void updateVertexBuffer(Path path) {
        ByteBuffer goalVertexByteBuffer = ByteBuffer.allocateDirect(path.getPoses().size() * 3 * 32);
        goalVertexByteBuffer.order(ByteOrder.nativeOrder());
        this.vertexBuffer = goalVertexByteBuffer.asFloatBuffer();
        int i = 0;
        if (path.getPoses().size() > 0) {
            this.frame = GraphName.of(path.getPoses().get(0).getHeader().getFrameId());
            for (PoseStamped pose : path.getPoses()) {
                this.vertexBuffer.put((float) pose.getPose().getPosition().getX());
                this.vertexBuffer.put((float) pose.getPose().getPosition().getY());
                this.vertexBuffer.put((float) pose.getPose().getPosition().getZ());
                i++;
            }
        }
        this.vertexBuffer.position(0);
        this.numPoints = i;
    }

    public GraphName getFrame() {
        return this.frame;
    }
}
