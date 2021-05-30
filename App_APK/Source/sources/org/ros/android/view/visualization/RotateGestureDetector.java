package org.ros.android.view.visualization;

import android.view.MotionEvent;
import org.ros.math.RosMath;

public class RotateGestureDetector {
    private static final double MAX_DELTA_ANGLE = 0.1d;
    private final OnRotateGestureListener listener;
    private MotionEvent previousMotionEvent;

    public interface OnRotateGestureListener {
        boolean onRotate(MotionEvent motionEvent, MotionEvent motionEvent2, double d);
    }

    public RotateGestureDetector(OnRotateGestureListener listener2) {
        this.listener = listener2;
    }

    private double angle(MotionEvent event) {
        return Math.atan2((double) (event.getY(0) - event.getY(1)), (double) (event.getX(0) - event.getX(1)));
    }

    public boolean onTouchEvent(MotionEvent event) {
        if (event.getPointerCount() != 2) {
            return false;
        }
        if (this.previousMotionEvent == null) {
            this.previousMotionEvent = MotionEvent.obtain(event);
            return false;
        }
        boolean result = this.listener.onRotate(this.previousMotionEvent, event, RosMath.clamp(angle(this.previousMotionEvent) - angle(event), -0.1d, (double) MAX_DELTA_ANGLE));
        this.previousMotionEvent.recycle();
        this.previousMotionEvent = MotionEvent.obtain(event);
        return result;
    }
}
