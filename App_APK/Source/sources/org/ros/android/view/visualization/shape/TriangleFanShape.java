package org.ros.android.view.visualization.shape;

import java.nio.FloatBuffer;
import javax.microedition.khronos.opengles.GL10;
import org.ros.android.view.visualization.Color;
import org.ros.android.view.visualization.Vertices;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.rosjava_geometry.Transform;

public class TriangleFanShape extends BaseShape {
    private final FloatBuffer vertices;

    public /* bridge */ /* synthetic */ void draw(VisualizationView visualizationView, GL10 gl10) {
        super.draw(visualizationView, gl10);
    }

    public /* bridge */ /* synthetic */ Color getColor() {
        return super.getColor();
    }

    public /* bridge */ /* synthetic */ Transform getTransform() {
        return super.getTransform();
    }

    public /* bridge */ /* synthetic */ void setColor(Color color) {
        super.setColor(color);
    }

    public /* bridge */ /* synthetic */ void setTransform(Transform transform) {
        super.setTransform(transform);
    }

    public TriangleFanShape(float[] vertices2, Color color) {
        this.vertices = Vertices.toFloatBuffer(vertices2);
        setColor(color);
    }

    public void drawShape(VisualizationView view, GL10 gl) {
        Vertices.drawTriangleFan(gl, this.vertices, getColor());
    }
}
