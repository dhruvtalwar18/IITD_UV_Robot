package org.ros.android.view.visualization;

import com.google.common.base.Preconditions;
import javax.microedition.khronos.opengles.GL10;

public class Color {
    private float alpha;
    private float blue;
    private float green;
    private float red;

    public static Color copyOf(Color color) {
        return new Color(color.red, color.green, color.blue, color.alpha);
    }

    public static Color fromHexAndAlpha(String hex, float alpha2) {
        Preconditions.checkArgument(hex.length() == 6);
        return new Color(((float) Integer.parseInt(hex.substring(0, 2), 16)) / 255.0f, ((float) Integer.parseInt(hex.substring(2, 4), 16)) / 255.0f, ((float) Integer.parseInt(hex.substring(4), 16)) / 255.0f, alpha2);
    }

    public Color(float red2, float green2, float blue2, float alpha2) {
        boolean z = true;
        Preconditions.checkArgument(0.0f <= red2 && red2 <= 1.0f);
        Preconditions.checkArgument(0.0f <= green2 && green2 <= 1.0f);
        Preconditions.checkArgument(0.0f <= blue2 && blue2 <= 1.0f);
        Preconditions.checkArgument((0.0f > alpha2 || alpha2 > 1.0f) ? false : z);
        this.red = red2;
        this.green = green2;
        this.blue = blue2;
        this.alpha = alpha2;
    }

    public void apply(GL10 gl) {
        gl.glColor4f(this.red, this.green, this.blue, this.alpha);
    }

    public float getRed() {
        return this.red;
    }

    public void setRed(float red2) {
        this.red = red2;
    }

    public float getGreen() {
        return this.green;
    }

    public void setGreen(float green2) {
        this.green = green2;
    }

    public float getBlue() {
        return this.blue;
    }

    public void setBlue(float blue2) {
        this.blue = blue2;
    }

    public float getAlpha() {
        return this.alpha;
    }

    public void setAlpha(float alpha2) {
        this.alpha = alpha2;
    }
}
