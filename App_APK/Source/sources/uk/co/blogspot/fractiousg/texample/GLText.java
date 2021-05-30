package uk.co.blogspot.fractiousg.texample;

import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Typeface;
import android.opengl.GLUtils;
import android.util.Log;
import javax.microedition.khronos.opengles.GL10;

public class GLText {
    public static final int CHAR_BATCH_SIZE = 100;
    public static final int CHAR_CNT = 96;
    public static final int CHAR_END = 126;
    public static final int CHAR_NONE = 32;
    public static final int CHAR_START = 32;
    public static final int CHAR_UNKNOWN = 95;
    public static final int FONT_SIZE_MAX = 180;
    public static final int FONT_SIZE_MIN = 6;
    AssetManager assets;
    SpriteBatch batch;
    int cellHeight = 0;
    int cellWidth = 0;
    float charHeight = 0.0f;
    TextureRegion[] charRgn = new TextureRegion[96];
    float charWidthMax = 0.0f;
    final float[] charWidths = new float[96];
    int colCnt = 0;
    float fontAscent = 0.0f;
    float fontDescent = 0.0f;
    float fontHeight = 0.0f;
    int fontPadX = 0;
    int fontPadY = 0;
    GL10 gl;
    int rowCnt = 0;
    float scaleX = 1.0f;
    float scaleY = 1.0f;
    float spaceX = 0.0f;
    int textureId = -1;
    TextureRegion textureRgn;
    int textureSize = 0;

    public GLText(GL10 gl2, AssetManager assets2) {
        this.gl = gl2;
        this.assets = assets2;
        this.batch = new SpriteBatch(gl2, 100);
    }

    public boolean load(String file, int size, int padX, int padY) {
        return load(Typeface.createFromAsset(this.assets, file), size, padX, padY);
    }

    public boolean load(Typeface tf, int size, int padX, int padY) {
        this.fontPadX = padX;
        this.fontPadY = padY;
        Paint paint = new Paint();
        paint.setAntiAlias(true);
        paint.setTextSize((float) size);
        paint.setColor(-1);
        paint.setTypeface(tf);
        Paint.FontMetrics fm = paint.getFontMetrics();
        this.fontHeight = (float) Math.ceil((double) (Math.abs(fm.bottom) + Math.abs(fm.top)));
        this.fontAscent = (float) Math.ceil((double) Math.abs(fm.ascent));
        this.fontDescent = (float) Math.ceil((double) Math.abs(fm.descent));
        char[] s = new char[2];
        this.charHeight = 0.0f;
        this.charWidthMax = 0.0f;
        float[] w = new float[2];
        int cnt = 0;
        for (char c = ' '; c <= '~'; c = (char) (c + 1)) {
            s[0] = c;
            paint.getTextWidths(s, 0, 1, w);
            this.charWidths[cnt] = w[0];
            if (this.charWidths[cnt] > this.charWidthMax) {
                this.charWidthMax = this.charWidths[cnt];
            }
            cnt++;
        }
        s[0] = ' ';
        paint.getTextWidths(s, 0, 1, w);
        this.charWidths[cnt] = w[0];
        if (this.charWidths[cnt] > this.charWidthMax) {
            this.charWidthMax = this.charWidths[cnt];
        }
        int i = cnt + 1;
        this.charHeight = this.fontHeight;
        this.cellWidth = ((int) this.charWidthMax) + (this.fontPadX * 2);
        this.cellHeight = ((int) this.charHeight) + (this.fontPadY * 2);
        int maxSize = this.cellWidth > this.cellHeight ? this.cellWidth : this.cellHeight;
        if (maxSize < 6) {
            int i2 = maxSize;
            float[] fArr = w;
            char[] cArr = s;
            return false;
        } else if (maxSize > 180) {
            Paint paint2 = paint;
            int i3 = maxSize;
            float[] fArr2 = w;
            char[] cArr2 = s;
            return false;
        } else {
            if (maxSize <= 24) {
                this.textureSize = 256;
            } else if (maxSize <= 40) {
                this.textureSize = 512;
            } else if (maxSize <= 80) {
                this.textureSize = 1024;
            } else {
                this.textureSize = 2048;
            }
            Bitmap bitmap = Bitmap.createBitmap(this.textureSize, this.textureSize, Bitmap.Config.ALPHA_8);
            Canvas canvas = new Canvas(bitmap);
            bitmap.eraseColor(0);
            this.colCnt = this.textureSize / this.cellWidth;
            float[] fArr3 = w;
            this.rowCnt = (int) Math.ceil((double) (96.0f / ((float) this.colCnt)));
            float x = (float) this.fontPadX;
            float y = (((float) (this.cellHeight - 1)) - this.fontDescent) - ((float) this.fontPadY);
            char c2 = ' ';
            while (true) {
                char c3 = c2;
                if (c3 > '~') {
                    break;
                }
                s[0] = c3;
                Bitmap bitmap2 = bitmap;
                int maxSize2 = maxSize;
                char c4 = c3;
                canvas.drawText(s, 0, 1, x, y, paint);
                x += (float) this.cellWidth;
                if ((x + ((float) this.cellWidth)) - ((float) this.fontPadX) > ((float) this.textureSize)) {
                    y += (float) this.cellHeight;
                    x = (float) this.fontPadX;
                }
                c2 = (char) (c4 + 1);
                maxSize = maxSize2;
                bitmap = bitmap2;
            }
            Bitmap bitmap3 = bitmap;
            int i4 = maxSize;
            int c5 = 0;
            s[0] = ' ';
            char[] cArr3 = s;
            canvas.drawText(s, 0, 1, x, y, paint);
            int[] textureIds = new int[1];
            this.gl.glGenTextures(1, textureIds, 0);
            Log.i("text handle", "" + textureIds[0]);
            this.textureId = textureIds[0];
            this.gl.glBindTexture(3553, this.textureId);
            this.gl.glTexParameterf(3553, 10241, 9728.0f);
            this.gl.glTexParameterf(3553, 10240, 9729.0f);
            this.gl.glTexParameterf(3553, 10242, 33071.0f);
            this.gl.glTexParameterf(3553, 10243, 33071.0f);
            Bitmap bitmap4 = bitmap3;
            GLUtils.texImage2D(3553, 0, bitmap4, 0);
            this.gl.glBindTexture(3553, 0);
            bitmap4.recycle();
            float x2 = 0.0f;
            float y2 = 0.0f;
            while (true) {
                int c6 = c5;
                if (c6 < 96) {
                    Paint paint3 = paint;
                    this.charRgn[c6] = new TextureRegion((float) this.textureSize, (float) this.textureSize, x2, y2, (float) (this.cellWidth - 1), (float) (this.cellHeight - 1));
                    x2 += (float) this.cellWidth;
                    if (((float) this.cellWidth) + x2 > ((float) this.textureSize)) {
                        y2 += (float) this.cellHeight;
                        x2 = 0.0f;
                    }
                    c5 = c6 + 1;
                    paint = paint3;
                    int i5 = padX;
                    int i6 = padY;
                } else {
                    this.textureRgn = new TextureRegion((float) this.textureSize, (float) this.textureSize, 0.0f, 0.0f, (float) this.textureSize, (float) this.textureSize);
                    return true;
                }
            }
        }
    }

    public void begin() {
        begin(1.0f, 1.0f, 1.0f, 1.0f);
    }

    public void begin(float alpha) {
        begin(1.0f, 1.0f, 1.0f, alpha);
    }

    public void begin(float red, float green, float blue, float alpha) {
        this.gl.glColor4f(red, green, blue, alpha);
        this.gl.glBindTexture(3553, this.textureId);
        this.batch.beginBatch();
    }

    public void end() {
        this.batch.endBatch();
        this.gl.glBindTexture(3553, 0);
        this.gl.glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    }

    public void draw(String text, float x, float y) {
        float chrHeight = ((float) this.cellHeight) * this.scaleY;
        float chrWidth = ((float) this.cellWidth) * this.scaleX;
        int len = text.length();
        float x2 = x + ((chrWidth / 2.0f) - (((float) this.fontPadX) * this.scaleX));
        float y2 = y + ((chrHeight / 2.0f) - (((float) this.fontPadY) * this.scaleY));
        int i = 0;
        while (true) {
            int i2 = i;
            if (i2 < len) {
                int c = text.charAt(i2) - ' ';
                if (c < 0 || c >= 96) {
                    c = 95;
                }
                int c2 = c;
                this.batch.drawSprite(x2, y2, chrWidth, chrHeight, this.charRgn[c2]);
                x2 += (this.charWidths[c2] + this.spaceX) * this.scaleX;
                i = i2 + 1;
            } else {
                return;
            }
        }
    }

    public float drawC(String text, float x, float y) {
        float len = getLength(text);
        draw(text, x - (len / 2.0f), y - (getCharHeight() / 2.0f));
        return len;
    }

    public float drawCX(String text, float x, float y) {
        float len = getLength(text);
        draw(text, x - (len / 2.0f), y);
        return len;
    }

    public void drawCY(String text, float x, float y) {
        draw(text, x, y - (getCharHeight() / 2.0f));
    }

    public void setScale(float scale) {
        this.scaleY = scale;
        this.scaleX = scale;
    }

    public void setScale(float sx, float sy) {
        this.scaleX = sx;
        this.scaleY = sy;
    }

    public float getScaleX() {
        return this.scaleX;
    }

    public float getScaleY() {
        return this.scaleY;
    }

    public void setSpace(float space) {
        this.spaceX = space;
    }

    public float getSpace() {
        return this.spaceX;
    }

    public float getLength(String text) {
        float len = 0.0f;
        int strLen = text.length();
        for (int i = 0; i < strLen; i++) {
            len += this.charWidths[text.charAt(i) - ' '] * this.scaleX;
        }
        return len + (strLen > 1 ? ((float) (strLen - 1)) * this.spaceX * this.scaleX : 0.0f);
    }

    public float getCharWidth(char chr) {
        return this.charWidths[chr - ' '] * this.scaleX;
    }

    public float getCharWidthMax() {
        return this.charWidthMax * this.scaleX;
    }

    public float getCharHeight() {
        return this.charHeight * this.scaleY;
    }

    public float getAscent() {
        return this.fontAscent * this.scaleY;
    }

    public float getDescent() {
        return this.fontDescent * this.scaleY;
    }

    public float getHeight() {
        return this.fontHeight * this.scaleY;
    }
}
