package org.ros.android.view.visualization;

import javax.microedition.khronos.opengles.GL10;

public class Viewport {
    private final int height;
    private final int width;

    public Viewport(int width2, int height2) {
        this.width = width2;
        this.height = height2;
    }

    public void apply(GL10 gl) {
        gl.glViewport(0, 0, this.width, this.height);
        gl.glMatrixMode(5889);
        gl.glLoadIdentity();
        gl.glOrthof(((float) (-this.width)) / 2.0f, ((float) this.width) / 2.0f, ((float) (-this.height)) / 2.0f, ((float) this.height) / 2.0f, -10000.0f, 10000.0f);
    }

    public int getWidth() {
        return this.width;
    }

    public int getHeight() {
        return this.height;
    }
}
