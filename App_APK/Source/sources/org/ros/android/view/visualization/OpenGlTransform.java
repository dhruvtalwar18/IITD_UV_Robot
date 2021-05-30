package org.ros.android.view.visualization;

import java.nio.FloatBuffer;
import javax.microedition.khronos.opengles.GL10;
import org.ros.rosjava_geometry.Transform;

public class OpenGlTransform {
    private static final ThreadLocal<FloatBuffer> buffer = new ThreadLocal<FloatBuffer>() {
        /* access modifiers changed from: protected */
        public FloatBuffer initialValue() {
            return FloatBuffer.allocate(16);
        }

        public FloatBuffer get() {
            FloatBuffer buffer = (FloatBuffer) super.get();
            buffer.clear();
            return buffer;
        }
    };

    private OpenGlTransform() {
    }

    public static void apply(GL10 gl, Transform transform) {
        FloatBuffer matrix = buffer.get();
        for (double value : transform.toMatrix()) {
            matrix.put((float) value);
        }
        matrix.position(0);
        gl.glMultMatrixf(matrix);
    }
}
