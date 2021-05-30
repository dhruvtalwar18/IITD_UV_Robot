package org.ros.android.view.visualization.layer;

import com.google.common.base.Preconditions;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import javax.microedition.khronos.opengles.GL10;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.android.view.visualization.Color;
import org.ros.android.view.visualization.Vertices;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import sensor_msgs.PointCloud2;

public class PointCloud2DLayer extends SubscriberLayer<PointCloud2> implements TfLayer {
    private static final Color FREE_SPACE_COLOR = Color.fromHexAndAlpha("377dfa", 0.1f);
    private static final Color OCCUPIED_SPACE_COLOR = Color.fromHexAndAlpha("377dfa", 0.3f);
    private static final float POINT_SIZE = 10.0f;
    /* access modifiers changed from: private */
    public GraphName frame;
    private final Object mutex;
    private FloatBuffer vertexBackBuffer;
    private FloatBuffer vertexFrontBuffer;

    public PointCloud2DLayer(String topicName) {
        this(GraphName.of(topicName));
    }

    public PointCloud2DLayer(GraphName topicName) {
        super(topicName, PointCloud2._TYPE);
        this.mutex = new Object();
    }

    public void draw(VisualizationView view, GL10 gl) {
        if (this.vertexFrontBuffer != null) {
            synchronized (this.mutex) {
                Vertices.drawTriangleFan(gl, this.vertexFrontBuffer, FREE_SPACE_COLOR);
                FloatBuffer pointVertices = this.vertexFrontBuffer.duplicate();
                pointVertices.position(3);
                Vertices.drawPoints(gl, pointVertices, OCCUPIED_SPACE_COLOR, POINT_SIZE);
            }
        }
    }

    public void onStart(VisualizationView view, ConnectedNode connectedNode) {
        super.onStart(view, connectedNode);
        getSubscriber().addMessageListener(new MessageListener<PointCloud2>() {
            public void onNewMessage(PointCloud2 pointCloud) {
                GraphName unused = PointCloud2DLayer.this.frame = GraphName.of(pointCloud.getHeader().getFrameId());
                PointCloud2DLayer.this.updateVertexBuffer(pointCloud);
            }
        });
    }

    /* access modifiers changed from: private */
    public void updateVertexBuffer(PointCloud2 pointCloud) {
        Preconditions.checkArgument(pointCloud.getHeight() == 1);
        Preconditions.checkArgument(pointCloud.getIsDense());
        Preconditions.checkArgument(pointCloud.getFields().size() == 3);
        Preconditions.checkArgument(pointCloud.getFields().get(0).getDatatype() == 7);
        Preconditions.checkArgument(pointCloud.getFields().get(1).getDatatype() == 7);
        Preconditions.checkArgument(pointCloud.getFields().get(2).getDatatype() == 7);
        Preconditions.checkArgument(pointCloud.getPointStep() == 16);
        Preconditions.checkArgument(pointCloud.getData().order().equals(ByteOrder.LITTLE_ENDIAN));
        int size = ((pointCloud.getRowStep() / pointCloud.getPointStep()) + 1) * 3;
        if (this.vertexBackBuffer == null || this.vertexBackBuffer.capacity() < size) {
            this.vertexBackBuffer = Vertices.allocateBuffer(size);
        }
        this.vertexBackBuffer.clear();
        this.vertexBackBuffer.put(0.0f);
        this.vertexBackBuffer.put(0.0f);
        this.vertexBackBuffer.put(0.0f);
        ChannelBuffer buffer = pointCloud.getData();
        while (buffer.readable()) {
            this.vertexBackBuffer.put(buffer.readFloat());
            this.vertexBackBuffer.put(buffer.readFloat());
            this.vertexBackBuffer.put(0.0f);
            buffer.readFloat();
            buffer.readFloat();
        }
        this.vertexBackBuffer.position(0);
        synchronized (this.mutex) {
            FloatBuffer tmp = this.vertexFrontBuffer;
            this.vertexFrontBuffer = this.vertexBackBuffer;
            this.vertexBackBuffer = tmp;
        }
    }

    public GraphName getFrame() {
        return this.frame;
    }
}
