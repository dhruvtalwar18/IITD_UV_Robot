package org.ros.android.view.visualization.shape;

import com.google.common.base.Preconditions;
import javax.microedition.khronos.opengles.GL10;
import org.ros.android.view.visualization.Color;
import org.ros.android.view.visualization.OpenGlTransform;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.rosjava_geometry.Transform;

abstract class BaseShape implements Shape {
    private Color color;
    private Transform transform;

    /* access modifiers changed from: protected */
    public abstract void drawShape(VisualizationView visualizationView, GL10 gl10);

    public BaseShape() {
        setTransform(Transform.identity());
    }

    public void draw(VisualizationView view, GL10 gl) {
        gl.glPushMatrix();
        OpenGlTransform.apply(gl, getTransform());
        scale(view, gl);
        drawShape(view, gl);
        gl.glPopMatrix();
    }

    /* access modifiers changed from: protected */
    public void scale(VisualizationView view, GL10 gl) {
    }

    public Color getColor() {
        Preconditions.checkNotNull(this.color);
        return this.color;
    }

    public void setColor(Color color2) {
        this.color = color2;
    }

    public Transform getTransform() {
        Preconditions.checkNotNull(this.transform);
        return this.transform;
    }

    public void setTransform(Transform pose) {
        this.transform = pose;
    }
}
