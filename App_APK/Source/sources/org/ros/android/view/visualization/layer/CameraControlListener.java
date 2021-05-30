package org.ros.android.view.visualization.layer;

public interface CameraControlListener {
    void onDoubleTap(float f, float f2);

    void onRotate(float f, float f2, double d);

    void onTranslate(float f, float f2);

    void onZoom(float f, float f2, float f3);
}
