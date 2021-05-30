package org.ros.android.view.visualization.layer;

import javax.microedition.khronos.opengles.GL10;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.android.view.visualization.shape.PixelSpacePoseShape;
import org.ros.android.view.visualization.shape.Shape;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;

public class RobotLayer extends DefaultLayer implements TfLayer {
    private final GraphName frame;
    private Shape shape;

    public RobotLayer(GraphName frame2) {
        this.frame = frame2;
    }

    public RobotLayer(String frame2) {
        this(GraphName.of(frame2));
    }

    public void draw(VisualizationView view, GL10 gl) {
        if (this.shape != null) {
            this.shape.draw(view, gl);
        }
    }

    public void onStart(VisualizationView view, ConnectedNode connectedNode) {
        this.shape = new PixelSpacePoseShape();
    }

    public GraphName getFrame() {
        return this.frame;
    }
}
