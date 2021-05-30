package org.ros.android.view.visualization.layer;

import android.support.v4.view.GestureDetectorCompat;
import android.view.GestureDetector;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;
import com.google.common.base.Preconditions;
import org.ros.android.view.visualization.RotateGestureDetector;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.concurrent.ListenerGroup;
import org.ros.concurrent.SignalRunnable;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMainExecutor;

public class CameraControlLayer extends DefaultLayer {
    /* access modifiers changed from: private */
    public ListenerGroup<CameraControlListener> listeners;
    /* access modifiers changed from: private */
    public RotateGestureDetector rotateGestureDetector;
    /* access modifiers changed from: private */
    public GestureDetectorCompat translateGestureDetector;
    /* access modifiers changed from: private */
    public ScaleGestureDetector zoomGestureDetector;

    public void init(NodeMainExecutor nodeMainExecutor) {
        this.listeners = new ListenerGroup<>(nodeMainExecutor.getScheduledExecutorService());
    }

    public void addListener(CameraControlListener listener) {
        Preconditions.checkNotNull(this.listeners);
        this.listeners.add(listener);
    }

    public boolean onTouchEvent(VisualizationView view, MotionEvent event) {
        if (this.translateGestureDetector == null || this.rotateGestureDetector == null || this.zoomGestureDetector == null) {
            return false;
        }
        boolean translateGestureHandled = this.translateGestureDetector.onTouchEvent(event);
        boolean rotateGestureHandled = this.rotateGestureDetector.onTouchEvent(event);
        boolean zoomGestureHandled = this.zoomGestureDetector.onTouchEvent(event);
        if (translateGestureHandled || rotateGestureHandled || zoomGestureHandled || super.onTouchEvent(view, event)) {
            return true;
        }
        return false;
    }

    public void onStart(final VisualizationView view, ConnectedNode connectedNode) {
        view.post(new Runnable() {
            public void run() {
                GestureDetectorCompat unused = CameraControlLayer.this.translateGestureDetector = new GestureDetectorCompat(view.getContext(), new GestureDetector.SimpleOnGestureListener() {
                    public boolean onDown(MotionEvent e) {
                        return true;
                    }

                    public boolean onScroll(MotionEvent event1, MotionEvent event2, final float distanceX, final float distanceY) {
                        view.getCamera().translate((double) (-distanceX), (double) distanceY);
                        CameraControlLayer.this.listeners.signal(new SignalRunnable<CameraControlListener>() {
                            public void run(CameraControlListener listener) {
                                listener.onTranslate(-distanceX, distanceY);
                            }
                        });
                        return true;
                    }

                    public boolean onDoubleTap(final MotionEvent e) {
                        CameraControlLayer.this.listeners.signal(new SignalRunnable<CameraControlListener>() {
                            public void run(CameraControlListener listener) {
                                listener.onDoubleTap(e.getX(), e.getY());
                            }
                        });
                        return true;
                    }
                });
                RotateGestureDetector unused2 = CameraControlLayer.this.rotateGestureDetector = new RotateGestureDetector(new RotateGestureDetector.OnRotateGestureListener() {
                    public boolean onRotate(MotionEvent event1, MotionEvent event2, double deltaAngle) {
                        float focusX = (event1.getX(0) + event1.getX(1)) / 2.0f;
                        float focusY = (event1.getY(0) + event1.getY(1)) / 2.0f;
                        view.getCamera().rotate((double) focusX, (double) focusY, deltaAngle);
                        final float f = focusX;
                        final float f2 = focusY;
                        final double d = deltaAngle;
                        CameraControlLayer.this.listeners.signal(new SignalRunnable<CameraControlListener>() {
                            public void run(CameraControlListener listener) {
                                listener.onRotate(f, f2, d);
                            }
                        });
                        return true;
                    }
                });
                ScaleGestureDetector unused3 = CameraControlLayer.this.zoomGestureDetector = new ScaleGestureDetector(view.getContext(), new ScaleGestureDetector.SimpleOnScaleGestureListener() {
                    public boolean onScale(ScaleGestureDetector detector) {
                        if (!detector.isInProgress()) {
                            return false;
                        }
                        final float focusX = detector.getFocusX();
                        final float focusY = detector.getFocusY();
                        final float factor = detector.getScaleFactor();
                        view.getCamera().zoom((double) focusX, (double) focusY, (double) factor);
                        CameraControlLayer.this.listeners.signal(new SignalRunnable<CameraControlListener>() {
                            public void run(CameraControlListener listener) {
                                listener.onZoom(focusX, focusY, factor);
                            }
                        });
                        return true;
                    }
                });
            }
        });
    }
}
