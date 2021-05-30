package uk.co.blogspot.fractiousg.texample;

import javax.microedition.khronos.opengles.GL10;

public class SpriteBatch {
    static final int INDICES_PER_SPRITE = 6;
    static final int VERTEX_SIZE = 4;
    static final int VERTICES_PER_SPRITE = 4;
    int bufferIndex = 0;
    GL10 gl;
    int maxSprites;
    int numSprites;
    float[] vertexBuffer;
    Vertices vertices;

    public SpriteBatch(GL10 gl2, int maxSprites2) {
        this.gl = gl2;
        this.vertexBuffer = new float[(maxSprites2 * 4 * 4)];
        this.vertices = new Vertices(gl2, maxSprites2 * 4, maxSprites2 * 6, false, true, false);
        this.maxSprites = maxSprites2;
        this.numSprites = 0;
        short[] indices = new short[(maxSprites2 * 6)];
        int len = indices.length;
        short j = 0;
        int i = 0;
        while (i < len) {
            indices[i + 0] = (short) (j + 0);
            indices[i + 1] = (short) (j + 1);
            indices[i + 2] = (short) (j + 2);
            indices[i + 3] = (short) (j + 2);
            indices[i + 4] = (short) (j + 3);
            indices[i + 5] = (short) (j + 0);
            i += 6;
            j = (short) (j + 4);
        }
        this.vertices.setIndices(indices, 0, len);
    }

    public void beginBatch() {
        this.numSprites = 0;
        this.bufferIndex = 0;
    }

    public void endBatch() {
        if (this.numSprites > 0) {
            this.vertices.setVertices(this.vertexBuffer, 0, this.bufferIndex);
            this.vertices.bind();
            this.vertices.draw(4, 0, this.numSprites * 6);
            this.vertices.unbind();
        }
    }

    public void drawSprite(float x, float y, float width, float height, TextureRegion region) {
        if (this.numSprites == this.maxSprites) {
            endBatch();
            this.numSprites = 0;
            this.bufferIndex = 0;
        }
        float halfWidth = width / 2.0f;
        float halfHeight = height / 2.0f;
        float x1 = x - halfWidth;
        float y1 = y - halfHeight;
        float x2 = x + halfWidth;
        float y2 = y + halfHeight;
        float[] fArr = this.vertexBuffer;
        int i = this.bufferIndex;
        this.bufferIndex = i + 1;
        fArr[i] = x1;
        float[] fArr2 = this.vertexBuffer;
        int i2 = this.bufferIndex;
        this.bufferIndex = i2 + 1;
        fArr2[i2] = y1;
        float[] fArr3 = this.vertexBuffer;
        int i3 = this.bufferIndex;
        this.bufferIndex = i3 + 1;
        fArr3[i3] = region.u1;
        float[] fArr4 = this.vertexBuffer;
        int i4 = this.bufferIndex;
        this.bufferIndex = i4 + 1;
        fArr4[i4] = region.v2;
        float[] fArr5 = this.vertexBuffer;
        int i5 = this.bufferIndex;
        this.bufferIndex = i5 + 1;
        fArr5[i5] = x2;
        float[] fArr6 = this.vertexBuffer;
        int i6 = this.bufferIndex;
        this.bufferIndex = i6 + 1;
        fArr6[i6] = y1;
        float[] fArr7 = this.vertexBuffer;
        int i7 = this.bufferIndex;
        this.bufferIndex = i7 + 1;
        fArr7[i7] = region.u2;
        float[] fArr8 = this.vertexBuffer;
        int i8 = this.bufferIndex;
        this.bufferIndex = i8 + 1;
        fArr8[i8] = region.v2;
        float[] fArr9 = this.vertexBuffer;
        int i9 = this.bufferIndex;
        this.bufferIndex = i9 + 1;
        fArr9[i9] = x2;
        float[] fArr10 = this.vertexBuffer;
        int i10 = this.bufferIndex;
        this.bufferIndex = i10 + 1;
        fArr10[i10] = y2;
        float[] fArr11 = this.vertexBuffer;
        int i11 = this.bufferIndex;
        this.bufferIndex = i11 + 1;
        fArr11[i11] = region.u2;
        float[] fArr12 = this.vertexBuffer;
        int i12 = this.bufferIndex;
        this.bufferIndex = i12 + 1;
        fArr12[i12] = region.v1;
        float[] fArr13 = this.vertexBuffer;
        int i13 = this.bufferIndex;
        this.bufferIndex = i13 + 1;
        fArr13[i13] = x1;
        float[] fArr14 = this.vertexBuffer;
        int i14 = this.bufferIndex;
        this.bufferIndex = i14 + 1;
        fArr14[i14] = y2;
        float[] fArr15 = this.vertexBuffer;
        int i15 = this.bufferIndex;
        this.bufferIndex = i15 + 1;
        fArr15[i15] = region.u1;
        float[] fArr16 = this.vertexBuffer;
        int i16 = this.bufferIndex;
        this.bufferIndex = i16 + 1;
        fArr16[i16] = region.v1;
        this.numSprites++;
    }
}
