package org.ros.android.view.visualization.shape;

import android.graphics.Typeface;
import javax.microedition.khronos.opengles.GL10;
import org.ros.android.view.visualization.VisualizationView;
import uk.co.blogspot.fractiousg.texample.GLText;

public class TextShapeFactory {
    private final GLText glText;

    public TextShapeFactory(VisualizationView view, GL10 gl) {
        this.glText = new GLText(gl, view.getContext().getAssets());
    }

    public void loadFont(Typeface typeface, int size, int padX, int padY) {
        this.glText.load(typeface, size, padX, padY);
    }

    public void loadFont(String file, int size, int padX, int padY) {
        this.glText.load(file, size, padX, padY);
    }

    public TextShape newTextShape(String text) {
        return new TextShape(this.glText, text);
    }
}
