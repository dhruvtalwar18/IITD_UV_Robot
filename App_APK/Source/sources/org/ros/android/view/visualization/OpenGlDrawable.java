package org.ros.android.view.visualization;

import javax.microedition.khronos.opengles.GL10;

public interface OpenGlDrawable {
    void draw(VisualizationView visualizationView, GL10 gl10);
}
