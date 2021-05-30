package org.ros.android.view.visualization.shape;

import javax.microedition.khronos.opengles.GL10;
import org.ros.android.view.visualization.VisualizationView;

public class PixelSpacePoiShape extends MetricSpacePoiShape {
    private static final float PIXELS_PER_METER = 100.0f;

    /* access modifiers changed from: protected */
    public void scale(VisualizationView view, GL10 gl) {
        gl.glScalef(PIXELS_PER_METER, PIXELS_PER_METER, 1.0f);
        gl.glScalef(1.0f / ((float) view.getCamera().getZoom()), 1.0f / ((float) view.getCamera().getZoom()), 1.0f);
    }
}
