package uk.co.blogspot.fractiousg.texample;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.IntBuffer;
import java.nio.ShortBuffer;
import javax.microedition.khronos.opengles.GL10;

public class Vertices {
    static final int COLOR_CNT = 4;
    static final int INDEX_SIZE = 2;
    static final int NORMAL_CNT = 3;
    static final int POSITION_CNT_2D = 2;
    static final int POSITION_CNT_3D = 3;
    static final int TEXCOORD_CNT = 2;
    final GL10 gl;
    final boolean hasColor;
    final boolean hasNormals;
    final boolean hasTexCoords;
    final ShortBuffer indices;
    public int numIndices;
    public int numVertices;
    public final int positionCnt;
    final int[] tmpBuffer;
    public final int vertexSize;
    public final int vertexStride;
    final IntBuffer vertices;

    public Vertices(GL10 gl2, int maxVertices, int maxIndices, boolean hasColor2, boolean hasTexCoords2, boolean hasNormals2) {
        this(gl2, maxVertices, maxIndices, hasColor2, hasTexCoords2, hasNormals2, false);
    }

    public Vertices(GL10 gl2, int maxVertices, int maxIndices, boolean hasColor2, boolean hasTexCoords2, boolean hasNormals2, boolean use3D) {
        this.gl = gl2;
        this.hasColor = hasColor2;
        this.hasTexCoords = hasTexCoords2;
        this.hasNormals = hasNormals2;
        int i = 3;
        int i2 = 2;
        this.positionCnt = use3D ? 3 : 2;
        this.vertexStride = this.positionCnt + (hasColor2 ? 4 : 0) + (!hasTexCoords2 ? 0 : i2) + (!hasNormals2 ? 0 : i);
        this.vertexSize = this.vertexStride * 4;
        ByteBuffer buffer = ByteBuffer.allocateDirect(this.vertexSize * maxVertices);
        buffer.order(ByteOrder.nativeOrder());
        this.vertices = buffer.asIntBuffer();
        if (maxIndices > 0) {
            ByteBuffer buffer2 = ByteBuffer.allocateDirect(maxIndices * 2);
            buffer2.order(ByteOrder.nativeOrder());
            this.indices = buffer2.asShortBuffer();
        } else {
            this.indices = null;
        }
        this.numVertices = 0;
        this.numIndices = 0;
        this.tmpBuffer = new int[((this.vertexSize * maxVertices) / 4)];
    }

    public void setVertices(float[] vertices2, int offset, int length) {
        this.vertices.clear();
        int last = offset + length;
        int i = offset;
        int j = 0;
        while (i < last) {
            this.tmpBuffer[j] = Float.floatToRawIntBits(vertices2[i]);
            i++;
            j++;
        }
        this.vertices.put(this.tmpBuffer, 0, length);
        this.vertices.flip();
        this.numVertices = length / this.vertexStride;
    }

    public void setIndices(short[] indices2, int offset, int length) {
        this.indices.clear();
        this.indices.put(indices2, offset, length);
        this.indices.flip();
        this.numIndices = length;
    }

    public void bind() {
        this.gl.glEnableClientState(32884);
        int i = 0;
        this.vertices.position(0);
        this.gl.glVertexPointer(this.positionCnt, 5126, this.vertexSize, this.vertices);
        int i2 = 4;
        if (this.hasColor) {
            this.gl.glEnableClientState(32886);
            this.vertices.position(this.positionCnt);
            this.gl.glColorPointer(4, 5126, this.vertexSize, this.vertices);
        }
        if (this.hasTexCoords) {
            this.gl.glEnableClientState(32888);
            this.vertices.position(this.positionCnt + (this.hasColor ? 4 : 0));
            this.gl.glTexCoordPointer(2, 5126, this.vertexSize, this.vertices);
        }
        if (this.hasNormals) {
            this.gl.glEnableClientState(32885);
            IntBuffer intBuffer = this.vertices;
            int i3 = this.positionCnt;
            if (!this.hasColor) {
                i2 = 0;
            }
            int i4 = i3 + i2;
            if (this.hasTexCoords) {
                i = 2;
            }
            intBuffer.position(i4 + i);
            this.gl.glNormalPointer(5126, this.vertexSize, this.vertices);
        }
    }

    public void draw(int primitiveType, int offset, int numVertices2) {
        if (this.indices != null) {
            this.indices.position(offset);
            this.gl.glDrawElements(primitiveType, numVertices2, 5123, this.indices);
            return;
        }
        this.gl.glDrawArrays(primitiveType, offset, numVertices2);
    }

    public void unbind() {
        if (this.hasColor) {
            this.gl.glDisableClientState(32886);
        }
        if (this.hasTexCoords) {
            this.gl.glDisableClientState(32888);
        }
        if (this.hasNormals) {
            this.gl.glDisableClientState(32885);
        }
    }

    public void drawFull(int primitiveType, int offset, int numVertices2) {
        this.gl.glEnableClientState(32884);
        int i = 0;
        this.vertices.position(0);
        this.gl.glVertexPointer(this.positionCnt, 5126, this.vertexSize, this.vertices);
        if (this.hasColor) {
            this.gl.glEnableClientState(32886);
            this.vertices.position(this.positionCnt);
            this.gl.glColorPointer(4, 5126, this.vertexSize, this.vertices);
        }
        if (this.hasTexCoords) {
            this.gl.glEnableClientState(32888);
            IntBuffer intBuffer = this.vertices;
            int i2 = this.positionCnt;
            if (this.hasColor) {
                i = 4;
            }
            intBuffer.position(i2 + i);
            this.gl.glTexCoordPointer(2, 5126, this.vertexSize, this.vertices);
        }
        if (this.indices != null) {
            this.indices.position(offset);
            this.gl.glDrawElements(primitiveType, numVertices2, 5123, this.indices);
        } else {
            this.gl.glDrawArrays(primitiveType, offset, numVertices2);
        }
        if (this.hasTexCoords) {
            this.gl.glDisableClientState(32888);
        }
        if (this.hasColor) {
            this.gl.glDisableClientState(32886);
        }
    }

    /* access modifiers changed from: package-private */
    public void setVtxPosition(int vtxIdx, float x, float y) {
        int index = this.vertexStride * vtxIdx;
        this.vertices.put(index + 0, Float.floatToRawIntBits(x));
        this.vertices.put(index + 1, Float.floatToRawIntBits(y));
    }

    /* access modifiers changed from: package-private */
    public void setVtxPosition(int vtxIdx, float x, float y, float z) {
        int index = this.vertexStride * vtxIdx;
        this.vertices.put(index + 0, Float.floatToRawIntBits(x));
        this.vertices.put(index + 1, Float.floatToRawIntBits(y));
        this.vertices.put(index + 2, Float.floatToRawIntBits(z));
    }

    /* access modifiers changed from: package-private */
    public void setVtxColor(int vtxIdx, float r, float g, float b, float a) {
        int index = (this.vertexStride * vtxIdx) + this.positionCnt;
        this.vertices.put(index + 0, Float.floatToRawIntBits(r));
        this.vertices.put(index + 1, Float.floatToRawIntBits(g));
        this.vertices.put(index + 2, Float.floatToRawIntBits(b));
        this.vertices.put(index + 3, Float.floatToRawIntBits(a));
    }

    /* access modifiers changed from: package-private */
    public void setVtxColor(int vtxIdx, float r, float g, float b) {
        int index = (this.vertexStride * vtxIdx) + this.positionCnt;
        this.vertices.put(index + 0, Float.floatToRawIntBits(r));
        this.vertices.put(index + 1, Float.floatToRawIntBits(g));
        this.vertices.put(index + 2, Float.floatToRawIntBits(b));
    }

    /* access modifiers changed from: package-private */
    public void setVtxColor(int vtxIdx, float a) {
        this.vertices.put((this.vertexStride * vtxIdx) + this.positionCnt + 3, Float.floatToRawIntBits(a));
    }

    /* access modifiers changed from: package-private */
    public void setVtxTexCoords(int vtxIdx, float u, float v) {
        int index = (this.vertexStride * vtxIdx) + this.positionCnt + (this.hasColor ? 4 : 0);
        this.vertices.put(index + 0, Float.floatToRawIntBits(u));
        this.vertices.put(index + 1, Float.floatToRawIntBits(v));
    }

    /* access modifiers changed from: package-private */
    public void setVtxNormal(int vtxIdx, float x, float y, float z) {
        int i = 0;
        int i2 = (this.vertexStride * vtxIdx) + this.positionCnt + (this.hasColor ? 4 : 0);
        if (this.hasTexCoords) {
            i = 2;
        }
        int index = i2 + i;
        this.vertices.put(index + 0, Float.floatToRawIntBits(x));
        this.vertices.put(index + 1, Float.floatToRawIntBits(y));
        this.vertices.put(index + 2, Float.floatToRawIntBits(z));
    }
}
