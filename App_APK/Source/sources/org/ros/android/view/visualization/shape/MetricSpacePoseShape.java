package org.ros.android.view.visualization.shape;

import org.ros.android.view.visualization.Color;

public class MetricSpacePoseShape extends TriangleFanShape {
    private static final Color COLOR = Color.fromHexAndAlpha("377dfa", 1.0f);
    private static final float[] VERTICES = {0.2f, 0.0f, 0.0f, -0.2f, -0.15f, 0.0f, -0.05f, 0.0f, 0.0f, -0.2f, 0.15f, 0.0f, 0.2f, 0.0f, 0.0f};

    public MetricSpacePoseShape() {
        super(VERTICES, COLOR);
    }
}
