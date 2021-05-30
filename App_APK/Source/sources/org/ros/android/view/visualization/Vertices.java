package org.ros.android.view.visualization;

import com.google.common.base.Preconditions;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import javax.microedition.khronos.opengles.GL10;

public class Vertices {
    private static final int FLOAT_BYTE_SIZE = 4;

    private Vertices() {
    }

    public static FloatBuffer allocateBuffer(int size) {
        ByteBuffer byteBuffer = ByteBuffer.allocateDirect(size * 4);
        byteBuffer.order(ByteOrder.nativeOrder());
        return byteBuffer.asFloatBuffer();
    }

    public static FloatBuffer toFloatBuffer(float[] floats) {
        FloatBuffer floatBuffer = allocateBuffer(floats.length);
        floatBuffer.put(floats);
        floatBuffer.position(0);
        return floatBuffer;
    }

    public static void drawPoints(GL10 gl, FloatBuffer vertices, Color color, float size) {
        vertices.mark();
        color.apply(gl);
        gl.glPointSize(size);
        gl.glEnableClientState(32884);
        gl.glVertexPointer(3, 5126, 0, vertices);
        gl.glDrawArrays(0, 0, countVertices(vertices, 3));
        gl.glDisableClientState(32884);
        vertices.reset();
    }

    public static void drawTriangleFan(GL10 gl, FloatBuffer vertices, Color color) {
        vertices.mark();
        color.apply(gl);
        gl.glEnableClientState(32884);
        gl.glVertexPointer(3, 5126, 0, vertices);
        gl.glDrawArrays(6, 0, countVertices(vertices, 3));
        gl.glDisableClientState(32884);
        vertices.reset();
    }

    public static void drawLineLoop(GL10 gl, FloatBuffer vertices, Color color, float width) {
        vertices.mark();
        color.apply(gl);
        gl.glLineWidth(width);
        gl.glEnableClientState(32884);
        gl.glVertexPointer(3, 5126, 0, vertices);
        gl.glDrawArrays(2, 0, countVertices(vertices, 3));
        gl.glDisableClientState(32884);
        vertices.reset();
    }

    public static void drawLines(GL10 gl, FloatBuffer vertices, Color color, float width) {
        vertices.mark();
        color.apply(gl);
        gl.glLineWidth(width);
        gl.glEnableClientState(32884);
        gl.glVertexPointer(3, 5126, 0, vertices);
        gl.glDrawArrays(1, 0, countVertices(vertices, 3));
        gl.glDisableClientState(32884);
        vertices.reset();
    }

    private static int countVertices(FloatBuffer vertices, int size) {
        boolean z = vertices.remaining() % size == 0;
        Preconditions.checkArgument(z, "Number of vertices: " + vertices.remaining());
        return vertices.remaining() / size;
    }
}
