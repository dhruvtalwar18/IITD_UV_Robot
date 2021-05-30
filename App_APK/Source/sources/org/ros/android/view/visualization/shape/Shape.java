package org.ros.android.view.visualization.shape;

import org.ros.android.view.visualization.Color;
import org.ros.android.view.visualization.OpenGlDrawable;
import org.ros.rosjava_geometry.Transform;

public interface Shape extends OpenGlDrawable {
    Color getColor();

    Transform getTransform();

    void setColor(Color color);

    void setTransform(Transform transform);
}
