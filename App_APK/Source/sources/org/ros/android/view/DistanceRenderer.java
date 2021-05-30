package org.ros.android.view;

import android.content.Context;
import android.content.SharedPreferences;
import android.opengl.GLSurfaceView;
import android.opengl.GLU;
import android.preference.PreferenceManager;
import java.util.List;
import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

class DistanceRenderer implements GLSurfaceView.Renderer {
    private static final float DISTANCE_VIEW_FIELD_OF_VIEW = 60.0f;
    private static final String DISTANCE_VIEW_ZOOM_LOCK_KEY = "DISTANCE_VIEW_ZOOM_LOCK";
    private static final String DISTANCE_VIEW_ZOOM_MODE_KEY = "DISTANCE_VIEW_ZOOM_MODE";
    private static final double DISTANCE_VIEW_ZOOM_MULTIPLIER = (1.0d / Math.tan(Math.toRadians(30.0d)));
    private static final String DISTANCE_VIEW_ZOOM_VALUE_KEY = "DISTANCE_VIEW_ZOOM_VALUE";
    private static final float MAX_DISTANCE_ZOOM = -13.856406f;
    private static final float MAX_FOV_DISTANCE = 8.0f;
    private static final float MIN_DISTANCE_ZOOM = -5.196152f;
    private static final float MIN_FOV_DISTANCE = 3.0f;
    private SharedPreferences.Editor editor;
    private DistancePoints rangeLines;
    private SharedPreferences sharedPreferences;
    private float theta;
    private float zoom = MAX_DISTANCE_ZOOM;
    private boolean zoomLocked;
    private ZoomMode zoomMode = ZoomMode.CLUTTER_ZOOM_MODE;

    DistanceRenderer() {
    }

    public void onSurfaceChanged(GL10 gl, int w, int h) {
        gl.glViewport(0, 0, w, h);
        gl.glMatrixMode(5889);
        gl.glLoadIdentity();
        GLU.gluPerspective(gl, DISTANCE_VIEW_FIELD_OF_VIEW, ((float) w) / ((float) h), 0.1f, 100.0f);
        gl.glMatrixMode(5888);
        gl.glLoadIdentity();
        gl.glHint(3155, 4354);
    }

    public void onSurfaceCreated(GL10 gl, EGLConfig arg1) {
        gl.glClearColor(0.0f, 0.0f, 0.0f, 0.5f);
        this.rangeLines = new DistancePoints();
    }

    public void onDrawFrame(GL10 gl) {
        gl.glClear(16384);
        gl.glLoadIdentity();
        gl.glTranslatef(0.0f, 0.0f, this.zoom);
        this.rangeLines.drawRange(gl);
        this.rangeLines.drawReferenceMarker(gl);
        this.rangeLines.drawRobot(gl);
        gl.glRotatef(this.theta, 0.0f, 0.0f, -1.0f);
    }

    public void setRotation(float theta2) {
        this.theta = theta2;
    }

    public void setNormalizedZoom(float normalizedZoomValue) {
        if (this.zoomMode == ZoomMode.CUSTOM_ZOOM_MODE) {
            setZoom(((1.0f - normalizedZoomValue) * 5.0f) + MIN_FOV_DISTANCE);
        }
    }

    public void setZoomMode(ZoomMode mode) {
        this.zoomMode = mode;
    }

    public void lockZoom() {
        this.zoomLocked = true;
    }

    public void unlockZoom() {
        this.zoomLocked = false;
    }

    public void currentSpeed(double speed) {
        if (this.zoomMode == ZoomMode.VELOCITY_ZOOM_MODE) {
            setZoom((float) (Math.abs(speed) * 8.0d));
        }
    }

    public void updateRange(List<Float> range, float maxRange, float minRange, float minTh, float thIncrement, float minDistToObject) {
        if (this.zoomMode == ZoomMode.CLUTTER_ZOOM_MODE) {
            setZoom(1.25f * minDistToObject);
        }
        this.rangeLines.updateRange(range, maxRange, minRange, minTh, thIncrement);
    }

    public void loadPreferences(Context context) {
        this.sharedPreferences = PreferenceManager.getDefaultSharedPreferences(context);
        int tmpMode = this.sharedPreferences.getInt(DISTANCE_VIEW_ZOOM_MODE_KEY, ZoomMode.CUSTOM_ZOOM_MODE.ordinal());
        if (tmpMode == ZoomMode.CUSTOM_ZOOM_MODE.ordinal()) {
            this.zoomMode = ZoomMode.CUSTOM_ZOOM_MODE;
        } else if (tmpMode == ZoomMode.CLUTTER_ZOOM_MODE.ordinal()) {
            this.zoomMode = ZoomMode.CLUTTER_ZOOM_MODE;
        } else if (tmpMode == ZoomMode.VELOCITY_ZOOM_MODE.ordinal()) {
            this.zoomMode = ZoomMode.VELOCITY_ZOOM_MODE;
        }
        this.zoomLocked = this.sharedPreferences.getBoolean(DISTANCE_VIEW_ZOOM_LOCK_KEY, false);
        this.editor = this.sharedPreferences.edit();
    }

    public void savePreferences(Context context) {
        this.editor = this.sharedPreferences.edit();
        this.editor.putInt(DISTANCE_VIEW_ZOOM_MODE_KEY, this.zoomMode.ordinal());
        this.editor.putFloat(DISTANCE_VIEW_ZOOM_VALUE_KEY, this.zoom);
        this.editor.putBoolean(DISTANCE_VIEW_ZOOM_LOCK_KEY, this.zoomLocked);
        this.editor.apply();
    }

    private void setZoom(float distanceFromCenter) {
        if (this.zoomLocked) {
            return;
        }
        if (distanceFromCenter < MIN_FOV_DISTANCE) {
            this.zoom = MIN_DISTANCE_ZOOM;
        } else if (distanceFromCenter > MAX_FOV_DISTANCE) {
            this.zoom = MAX_DISTANCE_ZOOM;
        } else {
            double d = (double) (-distanceFromCenter);
            double d2 = DISTANCE_VIEW_ZOOM_MULTIPLIER;
            Double.isNaN(d);
            this.zoom = (float) (d * d2);
        }
    }
}
