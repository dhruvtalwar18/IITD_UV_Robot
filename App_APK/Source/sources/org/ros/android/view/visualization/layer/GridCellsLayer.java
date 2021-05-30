package org.ros.android.view.visualization.layer;

import geometry_msgs.Point;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import javax.microedition.khronos.opengles.GL10;
import nav_msgs.GridCells;
import org.ros.android.view.visualization.Color;
import org.ros.android.view.visualization.Vertices;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.android.view.visualization.XYOrthographicCamera;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;

public class GridCellsLayer extends SubscriberLayer<GridCells> implements TfLayer {
    private XYOrthographicCamera camera;
    private final Color color;
    /* access modifiers changed from: private */
    public GraphName frame;
    /* access modifiers changed from: private */
    public final Lock lock;
    /* access modifiers changed from: private */
    public GridCells message;
    /* access modifiers changed from: private */
    public boolean ready;

    public GridCellsLayer(String topicName, Color color2) {
        this(GraphName.of(topicName), color2);
    }

    public GridCellsLayer(GraphName topicName, Color color2) {
        super(topicName, GridCells._TYPE);
        this.color = color2;
        this.frame = null;
        this.lock = new ReentrantLock();
        this.ready = false;
    }

    public void draw(VisualizationView view, GL10 gl) {
        if (this.ready) {
            super.draw(view, gl);
            this.lock.lock();
            double max = (double) Math.max(this.message.getCellWidth(), this.message.getCellHeight());
            double zoom = this.camera.getZoom();
            Double.isNaN(max);
            float pointSize = (float) (max * zoom);
            float[] vertices = new float[(this.message.getCells().size() * 3)];
            int i = 0;
            for (Point cell : this.message.getCells()) {
                vertices[i] = (float) cell.getX();
                vertices[i + 1] = (float) cell.getY();
                vertices[i + 2] = 0.0f;
                i += 3;
            }
            gl.glEnableClientState(32884);
            gl.glVertexPointer(3, 5126, 0, Vertices.toFloatBuffer(vertices));
            this.color.apply(gl);
            gl.glPointSize(pointSize);
            gl.glDrawArrays(0, 0, this.message.getCells().size());
            gl.glDisableClientState(32884);
            this.lock.unlock();
        }
    }

    public void onStart(final VisualizationView view, ConnectedNode connectedNode) {
        super.onStart(view, connectedNode);
        getSubscriber().addMessageListener(new MessageListener<GridCells>() {
            public void onNewMessage(GridCells data) {
                GraphName unused = GridCellsLayer.this.frame = GraphName.of(data.getHeader().getFrameId());
                if (view.getFrameTransformTree().lookUp(GridCellsLayer.this.frame) != null && GridCellsLayer.this.lock.tryLock()) {
                    GridCells unused2 = GridCellsLayer.this.message = data;
                    boolean unused3 = GridCellsLayer.this.ready = true;
                    GridCellsLayer.this.lock.unlock();
                }
            }
        });
    }

    public GraphName getFrame() {
        return this.frame;
    }
}
