package org.ros.android.view.visualization.shape;

import org.ros.android.view.visualization.Color;

public class GoalShape extends MetricSpacePoseShape {
    private static final Color COLOR = Color.fromHexAndAlpha("03d5c9", 0.3f);

    public GoalShape() {
        setColor(COLOR);
    }
}
