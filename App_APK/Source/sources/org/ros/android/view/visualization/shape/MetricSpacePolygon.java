package org.ros.android.view.visualization.shape;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import java.nio.FloatBuffer;
import java.util.List;
import javax.microedition.khronos.opengles.GL10;
import org.ros.android.view.visualization.Color;
import org.ros.android.view.visualization.Vertices;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.android.view.visualization.shape.Triangulate;
import org.ros.rosjava_geometry.Transform;

public class MetricSpacePolygon extends BaseShape {
    final List<FloatBuffer> triangles;
    final FloatBuffer vertexBuffer;

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

    public MetricSpacePolygon(float[] vertices, Color color) {
        this.vertexBuffer = Vertices.toFloatBuffer(vertices);
        setColor(color);
        List<Triangulate.Point> points = Lists.newArrayList();
        Triangulate.Point[] contour = new Triangulate.Point[(vertices.length / 3)];
        for (int i = 0; i < contour.length; i++) {
            contour[i] = new Triangulate.Point(vertices[i * 3], vertices[(i * 3) + 1]);
        }
        Preconditions.checkState(Triangulate.process(contour, points));
        this.triangles = Lists.newArrayList();
        for (int i2 = 0; i2 < points.size() / 3; i2++) {
            FloatBuffer triangle = Vertices.allocateBuffer(9);
            for (int j = i2 * 3; j < (i2 * 3) + 3; j++) {
                triangle.put(points.get(j).x());
                triangle.put(points.get(j).y());
                triangle.put(0.0f);
            }
            triangle.flip();
            this.triangles.add(triangle);
        }
    }

    public void drawShape(VisualizationView view, GL10 gl) {
        Color translucent = getColor();
        translucent.setAlpha(0.3f);
        for (FloatBuffer triangle : this.triangles) {
            Vertices.drawTriangleFan(gl, triangle, translucent);
        }
        Color opaque = getColor();
        opaque.setAlpha(1.0f);
        Vertices.drawLineLoop(gl, this.vertexBuffer, opaque, 3.0f);
        Vertices.drawPoints(gl, this.vertexBuffer, opaque, 10.0f);
    }
}
