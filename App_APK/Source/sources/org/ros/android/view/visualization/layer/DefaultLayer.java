package org.ros.android.view.visualization.layer;

import android.view.MotionEvent;
import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMainExecutor;

public abstract class DefaultLayer implements Layer {
    public void init(NodeMainExecutor nodeMainExecutor) {
    }

    public void draw(VisualizationView view, GL10 gl) {
    }

    public boolean onTouchEvent(VisualizationView view, MotionEvent event) {
        return false;
    }

    public void onStart(VisualizationView view, ConnectedNode connectedNode) {
    }

    public void onShutdown(VisualizationView view, Node node) {
    }

    public void onSurfaceChanged(VisualizationView view, GL10 gl, int width, int height) {
    }

    public void onSurfaceCreated(VisualizationView view, GL10 gl, EGLConfig config) {
    }
}
