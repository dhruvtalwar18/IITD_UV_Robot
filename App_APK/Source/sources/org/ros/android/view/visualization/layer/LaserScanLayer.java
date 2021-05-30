package org.ros.android.view.visualization.layer;

import java.nio.FloatBuffer;
import javax.microedition.khronos.opengles.GL10;
import org.ros.android.view.visualization.Color;
import org.ros.android.view.visualization.Vertices;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import sensor_msgs.LaserScan;

public class LaserScanLayer extends SubscriberLayer<LaserScan> implements TfLayer {
    private static final Color FREE_SPACE_COLOR = Color.fromHexAndAlpha("377dfa", 0.1f);
    private static final float LASER_SCAN_POINT_SIZE = 10.0f;
    private static final int LASER_SCAN_STRIDE = 15;
    private static final Color OCCUPIED_SPACE_COLOR = Color.fromHexAndAlpha("377dfa", 0.3f);
    /* access modifiers changed from: private */
    public GraphName frame;
    private final Object mutex;
    private FloatBuffer vertexBackBuffer;
    private FloatBuffer vertexFrontBuffer;

    public LaserScanLayer(String topicName) {
        this(GraphName.of(topicName));
    }

    public LaserScanLayer(GraphName topicName) {
        super(topicName, LaserScan._TYPE);
        this.mutex = new Object();
    }

    public void draw(VisualizationView view, GL10 gl) {
        if (this.vertexFrontBuffer != null) {
            synchronized (this.mutex) {
                Vertices.drawTriangleFan(gl, this.vertexFrontBuffer, FREE_SPACE_COLOR);
                FloatBuffer pointVertices = this.vertexFrontBuffer.duplicate();
                pointVertices.position(3);
                Vertices.drawPoints(gl, pointVertices, OCCUPIED_SPACE_COLOR, LASER_SCAN_POINT_SIZE);
            }
        }
    }

    public void onStart(VisualizationView view, ConnectedNode connectedNode) {
        super.onStart(view, connectedNode);
        getSubscriber().addMessageListener(new MessageListener<LaserScan>() {
            public void onNewMessage(LaserScan laserScan) {
                GraphName unused = LaserScanLayer.this.frame = GraphName.of(laserScan.getHeader().getFrameId());
                LaserScanLayer.this.updateVertexBuffer(laserScan, 15);
            }
        });
    }

    /* access modifiers changed from: private */
    public void updateVertexBuffer(LaserScan laserScan, int stride) {
        int size;
        float[] ranges;
        float maximumRange;
        int i = stride;
        float[] ranges2 = laserScan.getRanges();
        int size2 = ((ranges2.length / i) + 2) * 3;
        if (this.vertexBackBuffer == null || this.vertexBackBuffer.capacity() < size2) {
            this.vertexBackBuffer = Vertices.allocateBuffer(size2);
        }
        this.vertexBackBuffer.clear();
        this.vertexBackBuffer.put(0.0f);
        this.vertexBackBuffer.put(0.0f);
        this.vertexBackBuffer.put(0.0f);
        float minimumRange = laserScan.getRangeMin();
        float maximumRange2 = laserScan.getRangeMax();
        float angle = laserScan.getAngleMin();
        float angleIncrement = laserScan.getAngleIncrement();
        float angle2 = angle;
        int vertexCount = 0 + 1;
        int i2 = 0;
        while (i2 < ranges2.length) {
            float range = ranges2[i2];
            if (minimumRange >= range || range >= maximumRange2) {
                ranges = ranges2;
                size = size2;
                maximumRange = maximumRange2;
            } else {
                FloatBuffer floatBuffer = this.vertexBackBuffer;
                double d = (double) range;
                maximumRange = maximumRange2;
                double cos = Math.cos((double) angle2);
                Double.isNaN(d);
                floatBuffer.put((float) (d * cos));
                FloatBuffer floatBuffer2 = this.vertexBackBuffer;
                double d2 = (double) range;
                ranges = ranges2;
                size = size2;
                double sin = Math.sin((double) angle2);
                Double.isNaN(d2);
                floatBuffer2.put((float) (d2 * sin));
                this.vertexBackBuffer.put(0.0f);
                vertexCount++;
            }
            angle2 += ((float) i) * angleIncrement;
            i2 += i;
            maximumRange2 = maximumRange;
            ranges2 = ranges;
            size2 = size;
        }
        int i3 = size2;
        float f = maximumRange2;
        this.vertexBackBuffer.position(0);
        this.vertexBackBuffer.limit(vertexCount * 3);
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
