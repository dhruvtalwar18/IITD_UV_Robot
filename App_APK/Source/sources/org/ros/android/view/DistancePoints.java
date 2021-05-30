package org.ros.android.view;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.List;
import javax.microedition.khronos.opengles.GL10;

class DistancePoints {
    private float[] rangeVertexArray = new float[0];
    private FloatBuffer rangeVertexBuffer;
    private ByteBuffer rangeVertexByteBuffer;
    private int rangeVertexCount;
    private List<Float> rangeVertices = new ArrayList();
    private FloatBuffer referenceVertexBuffer;
    private FloatBuffer robotVertexBuffer;
    private int robotVertexCount;

    public DistancePoints() {
        initRobot();
        initReferenceMarker();
    }

    public void updateRange(List<Float> range, float maxRange, float minRange, float minimumTheta, float thetaIncrement) {
        this.rangeVertices.clear();
        double d = (double) minimumTheta;
        double radians = Math.toRadians(90.0d);
        Double.isNaN(d);
        float minimumTheta2 = (float) (d + radians);
        this.rangeVertices.add(Float.valueOf(0.0f));
        this.rangeVertices.add(Float.valueOf(0.0f));
        this.rangeVertices.add(Float.valueOf(0.0f));
        for (Float floatValue : range) {
            float rangeValue = floatValue.floatValue();
            if (rangeValue < maxRange && rangeValue > minRange) {
                List<Float> list = this.rangeVertices;
                double d2 = (double) rangeValue;
                double cos = Math.cos((double) minimumTheta2);
                Double.isNaN(d2);
                list.add(Float.valueOf((float) (d2 * cos)));
                List<Float> list2 = this.rangeVertices;
                double d3 = (double) rangeValue;
                double sin = Math.sin((double) minimumTheta2);
                Double.isNaN(d3);
                list2.add(Float.valueOf((float) (d3 * sin)));
                this.rangeVertices.add(Float.valueOf(0.0f));
            }
            minimumTheta2 += thetaIncrement;
        }
        if (this.rangeVertexArray.length != this.rangeVertices.size()) {
            this.rangeVertexArray = new float[this.rangeVertices.size()];
        }
        for (int i = 0; i < this.rangeVertices.size(); i++) {
            this.rangeVertexArray[i] = this.rangeVertices.get(i).floatValue();
        }
        this.rangeVertexCount = this.rangeVertexArray.length / 3;
        initRangeVertexBuffer();
        this.rangeVertexBuffer.put(this.rangeVertexArray);
        this.rangeVertexBuffer.position(0);
    }

    private void initRangeVertexBuffer() {
        int requiredVertexByteBufferCapacity = (this.rangeVertices.size() * 32) / 8;
        if (this.rangeVertexByteBuffer == null || requiredVertexByteBufferCapacity > this.rangeVertexByteBuffer.capacity()) {
            this.rangeVertexByteBuffer = ByteBuffer.allocateDirect(requiredVertexByteBufferCapacity);
            this.rangeVertexByteBuffer.order(ByteOrder.nativeOrder());
        }
        this.rangeVertexBuffer = this.rangeVertexByteBuffer.asFloatBuffer();
    }

    public void drawRange(GL10 gl) {
        try {
            gl.glDisable(2884);
            gl.glFrontFace(2304);
            gl.glVertexPointer(3, 5126, 0, this.rangeVertexBuffer);
            gl.glEnableClientState(32884);
            gl.glColor4f(0.35f, 0.35f, 0.35f, 0.7f);
            gl.glDrawArrays(6, 0, this.rangeVertexCount);
            gl.glPointSize(3.0f);
            gl.glColor4f(0.8f, 0.1f, 0.1f, 1.0f);
            gl.glDrawArrays(0, 1, this.rangeVertexCount - 1);
            gl.glDisableClientState(32884);
        } catch (NullPointerException e) {
        }
    }

    public void drawReferenceMarker(GL10 gl) {
        gl.glEnable(2848);
        gl.glEnableClientState(32884);
        gl.glVertexPointer(3, 5126, 0, this.referenceVertexBuffer);
        gl.glColor4f(0.7f, 0.7f, 0.7f, 1.0f);
        gl.glDrawArrays(1, 0, 10);
        gl.glDisableClientState(32884);
        gl.glDisable(2848);
    }

    public void drawRobot(GL10 gl) {
        gl.glEnable(2848);
        gl.glEnableClientState(32884);
        gl.glVertexPointer(3, 5126, 0, this.robotVertexBuffer);
        gl.glColor4f(0.6f, 0.0f, 0.0f, 1.0f);
        gl.glDrawArrays(2, 0, this.robotVertexCount);
        gl.glDisableClientState(32884);
        gl.glDisable(2848);
    }

    private void initRobot() {
        float[] robotVertices = {0.1651f, 0.1651f, 0.0f, 0.1651f, -0.1651f, 0.0f, -0.1651f, -0.1651f, 0.0f, -0.1651f, 0.1651f, 0.0f};
        this.robotVertexCount = robotVertices.length / 3;
        ByteBuffer vertexByteBuffer = ByteBuffer.allocateDirect((robotVertices.length * 32) / 8);
        vertexByteBuffer.order(ByteOrder.nativeOrder());
        this.robotVertexBuffer = vertexByteBuffer.asFloatBuffer();
        this.robotVertexBuffer.put(robotVertices);
        this.robotVertexBuffer.position(0);
    }

    private void initReferenceMarker() {
        float[] referenceVertices = {-1.5f, -2.0f, 0.0f, 1.5f, -2.0f, 0.0f, -1.5f, -2.0f - 0.25f, 0.0f, -1.5f, -2.0f + 0.25f, 0.0f, -0.5f, -2.0f - 0.25f, 0.0f, -0.5f, -2.0f + 0.25f, 0.0f, 0.5f, -2.0f - 0.25f, 0.0f, 0.5f, -2.0f + 0.25f, 0.0f, 1.5f, -2.0f - 0.25f, 0.0f, 1.5f, -2.0f + 0.25f, 0.0f};
        ByteBuffer referenceVertexByteBuffer = ByteBuffer.allocateDirect((referenceVertices.length * 32) / 8);
        referenceVertexByteBuffer.order(ByteOrder.nativeOrder());
        this.referenceVertexBuffer = referenceVertexByteBuffer.asFloatBuffer();
        this.referenceVertexBuffer.put(referenceVertices);
        this.referenceVertexBuffer.position(0);
    }
}
