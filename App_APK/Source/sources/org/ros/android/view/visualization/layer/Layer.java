package org.ros.android.view.visualization.layer;

import android.view.MotionEvent;
import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;
import org.ros.android.view.visualization.OpenGlDrawable;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMainExecutor;

public interface Layer extends OpenGlDrawable {
    void init(NodeMainExecutor nodeMainExecutor);

    void onShutdown(VisualizationView visualizationView, Node node);

    void onStart(VisualizationView visualizationView, ConnectedNode connectedNode);

    void onSurfaceChanged(VisualizationView visualizationView, GL10 gl10, int i, int i2);

    void onSurfaceCreated(VisualizationView visualizationView, GL10 gl10, EGLConfig eGLConfig);

    boolean onTouchEvent(VisualizationView visualizationView, MotionEvent motionEvent);
}
