package org.ros.android.view.visualization;

import android.opengl.GLSurfaceView;
import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;
import org.ros.android.view.visualization.layer.Layer;
import org.ros.android.view.visualization.layer.TfLayer;
import org.ros.namespace.GraphName;

public class XYOrthographicRenderer implements GLSurfaceView.Renderer {
    private static final Color BACKGROUND_COLOR = new Color(0.87f, 0.87f, 0.87f, 1.0f);
    private final VisualizationView view;

    public XYOrthographicRenderer(VisualizationView view2) {
        this.view = view2;
    }

    public void onSurfaceChanged(GL10 gl, int width, int height) {
        Viewport viewport = new Viewport(width, height);
        viewport.apply(gl);
        this.view.getCamera().setViewport(viewport);
        gl.glMatrixMode(5888);
        gl.glEnable(3042);
        gl.glBlendFunc(770, 771);
        gl.glDisable(2929);
        gl.glClearColor(BACKGROUND_COLOR.getRed(), BACKGROUND_COLOR.getGreen(), BACKGROUND_COLOR.getBlue(), BACKGROUND_COLOR.getAlpha());
        for (Layer layer : this.view.getLayers()) {
            layer.onSurfaceChanged(this.view, gl, width, height);
        }
    }

    public void onDrawFrame(GL10 gl) {
        gl.glClear(16384);
        gl.glLoadIdentity();
        this.view.getCamera().apply(gl);
        drawLayers(gl);
    }

    private void drawLayers(GL10 gl) {
        for (Layer layer : this.view.getLayers()) {
            gl.glPushMatrix();
            if (layer instanceof TfLayer) {
                GraphName layerFrame = ((TfLayer) layer).getFrame();
                if (layerFrame != null && this.view.getCamera().applyFrameTransform(gl, layerFrame)) {
                    layer.draw(this.view, gl);
                }
            } else {
                layer.draw(this.view, gl);
            }
            gl.glPopMatrix();
        }
    }

    public void onSurfaceCreated(GL10 gl, EGLConfig config) {
        for (Layer layer : this.view.getLayers()) {
            layer.onSurfaceCreated(this.view, gl, config);
        }
    }
}
