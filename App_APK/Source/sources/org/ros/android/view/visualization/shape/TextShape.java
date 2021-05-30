package org.ros.android.view.visualization.shape;

import java.nio.FloatBuffer;
import javax.microedition.khronos.opengles.GL10;
import org.ros.android.view.visualization.Color;
import org.ros.android.view.visualization.Vertices;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.rosjava_geometry.Transform;
import uk.co.blogspot.fractiousg.texample.GLText;

public class TextShape extends BaseShape {
    private final GLText glText;
    private FloatBuffer lines = Vertices.allocateBuffer(12);
    private final String text;
    private float x;
    private float y;

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

    public TextShape(GLText glText2, String text2) {
        this.glText = glText2;
        this.text = text2;
    }

    public void setOffset(float x2, float y2) {
        this.x = x2;
        this.y = y2;
        this.lines.put(0.0f);
        this.lines.put(0.0f);
        this.lines.put(0.0f);
        this.lines.put(x2);
        this.lines.put(y2);
        this.lines.put(0.0f);
        this.lines.put(x2);
        this.lines.put(y2);
        this.lines.put(0.0f);
        this.lines.put(this.glText.getLength(this.text) + x2);
        this.lines.put(y2);
        this.lines.put(0.0f);
        this.lines.flip();
    }

    /* access modifiers changed from: protected */
    public void scale(VisualizationView view, GL10 gl) {
        gl.glScalef(1.0f / ((float) view.getCamera().getZoom()), 1.0f / ((float) view.getCamera().getZoom()), 1.0f);
    }

    /* access modifiers changed from: protected */
    public void drawShape(VisualizationView view, GL10 gl) {
        Vertices.drawLines(gl, this.lines, getColor(), 3.0f);
        gl.glEnable(3553);
        this.glText.begin(getColor().getRed(), getColor().getGreen(), getColor().getBlue(), getColor().getAlpha());
        this.glText.draw(this.text, this.x, this.y);
        this.glText.end();
        gl.glDisable(3553);
    }
}
