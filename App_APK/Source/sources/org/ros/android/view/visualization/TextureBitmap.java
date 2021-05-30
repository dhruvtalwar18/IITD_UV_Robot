package org.ros.android.view.visualization;

import android.graphics.Bitmap;
import android.opengl.GLUtils;
import com.google.common.base.Preconditions;
import java.nio.FloatBuffer;
import javax.microedition.khronos.opengles.GL10;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.rosjava_geometry.Transform;

public class TextureBitmap implements OpenGlDrawable {
    public static final int HEIGHT = 1024;
    public static final int STRIDE = 1024;
    private Bitmap bitmapBack = Bitmap.createBitmap(1024, 1024, Bitmap.Config.ARGB_8888);
    private Bitmap bitmapFront = Bitmap.createBitmap(1024, 1024, Bitmap.Config.ARGB_8888);
    private int[] handle;
    private final Object mutex = new Object();
    private Transform origin;
    private final int[] pixels = new int[1048576];
    private boolean reload = true;
    private double scaledHeight;
    private double scaledWidth;
    private final FloatBuffer surfaceVertices = Vertices.toFloatBuffer(new float[]{0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f});
    private final FloatBuffer textureVertices = Vertices.toFloatBuffer(new float[]{0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f});

    public void updateFromPixelArray(int[] pixels2, int stride, float resolution, Transform origin2, int fillColor) {
        Preconditions.checkArgument(pixels2.length % stride == 0);
        int height = pixels2.length / stride;
        for (int y = 0; y < 1024; y++) {
            for (int x = 0; x < 1024; x++) {
                int sourceIndex = (y * stride) + x;
                int targetIndex = (y * 1024) + x;
                if (x >= stride || y >= height) {
                    this.pixels[targetIndex] = fillColor;
                } else {
                    this.pixels[targetIndex] = pixels2[sourceIndex];
                }
            }
        }
        update(origin2, stride, resolution, fillColor);
    }

    public void updateFromPixelBuffer(ChannelBuffer pixels2, int stride, float resolution, Transform origin2, int fillColor) {
        Preconditions.checkNotNull(pixels2);
        Preconditions.checkNotNull(origin2);
        int y = 0;
        int y2 = 0;
        while (y < 1024) {
            int i = y2;
            int x = 0;
            while (x < 1024) {
                if (x >= stride || !pixels2.readable()) {
                    this.pixels[i] = fillColor;
                } else {
                    this.pixels[i] = pixels2.readInt();
                }
                x++;
                i++;
            }
            y++;
            y2 = i;
        }
        update(origin2, stride, resolution, fillColor);
    }

    public void clearHandle() {
        this.handle = null;
    }

    private void update(Transform origin2, int stride, float resolution, int fillColor) {
        this.origin = origin2;
        this.scaledWidth = (double) (resolution * 1024.0f);
        this.scaledHeight = (double) (1024.0f * resolution);
        this.bitmapBack.setPixels(this.pixels, 0, 1024, 0, 0, 1024, 1024);
        synchronized (this.mutex) {
            Bitmap tmp = this.bitmapFront;
            this.bitmapFront = this.bitmapBack;
            this.bitmapBack = tmp;
            this.reload = true;
        }
    }

    private void bind(GL10 gl) {
        if (this.handle == null) {
            this.handle = new int[1];
            gl.glGenTextures(1, this.handle, 0);
            this.reload = true;
        }
        gl.glBindTexture(3553, this.handle[0]);
        gl.glTexParameterf(3553, 10241, 9728.0f);
        gl.glTexParameterf(3553, 10240, 9728.0f);
        synchronized (this.mutex) {
            if (this.reload) {
                GLUtils.texImage2D(3553, 0, this.bitmapFront, 0);
                this.reload = false;
            }
        }
    }

    public void draw(VisualizationView view, GL10 gl) {
        gl.glEnable(3553);
        bind(gl);
        gl.glPushMatrix();
        OpenGlTransform.apply(gl, this.origin);
        gl.glScalef((float) this.scaledWidth, (float) this.scaledHeight, 1.0f);
        gl.glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        gl.glEnableClientState(32884);
        gl.glEnableClientState(32888);
        gl.glVertexPointer(3, 5126, 0, this.surfaceVertices);
        gl.glTexCoordPointer(2, 5126, 0, this.textureVertices);
        gl.glDrawArrays(5, 0, 4);
        gl.glDisableClientState(32884);
        gl.glDisableClientState(32888);
        gl.glPopMatrix();
        gl.glBindTexture(3553, 0);
        gl.glDisable(3553);
    }
}
