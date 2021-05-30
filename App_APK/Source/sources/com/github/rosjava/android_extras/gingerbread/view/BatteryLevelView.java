package com.github.rosjava.android_extras.gingerbread.view;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.PorterDuff;
import android.graphics.PorterDuffColorFilter;
import android.graphics.Rect;
import android.support.v4.view.MotionEventCompat;
import android.util.AttributeSet;
import android.view.View;
import com.github.rosjava.android_extras.gingerbread.R;

public class BatteryLevelView extends View {
    private Paint gray;
    private Paint green;
    private float levelPercent;
    private Bitmap plug;
    private boolean pluggedIn;
    private Paint red;
    private Bitmap silhouette;
    private Paint yellow;

    public BatteryLevelView(Context ctx) {
        super(ctx);
        init(ctx);
    }

    public BatteryLevelView(Context context, AttributeSet attrs, int defStyle) {
        super(context, attrs, defStyle);
        init(context);
    }

    public BatteryLevelView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init(context);
    }

    private Paint makePaint(int color) {
        Paint paint = new Paint();
        paint.setColorFilter(new PorterDuffColorFilter(-16777216 | color, PorterDuff.Mode.SRC_ATOP));
        return paint;
    }

    private void init(Context context) {
        this.silhouette = BitmapFactory.decodeResource(context.getResources(), R.drawable.battery_silhouette);
        this.plug = BitmapFactory.decodeResource(context.getResources(), R.drawable.battery_charging);
        this.green = makePaint(MotionEventCompat.ACTION_POINTER_INDEX_MASK);
        this.yellow = makePaint(16776960);
        this.red = makePaint(16711680);
        this.gray = makePaint(8421504);
        this.levelPercent = 0.0f;
        this.pluggedIn = false;
    }

    public void setBatteryPercent(float percent) {
        this.levelPercent = percent;
        invalidate();
    }

    public void setPluggedIn(boolean plugged) {
        this.pluggedIn = plugged;
        invalidate();
    }

    /* access modifiers changed from: protected */
    public void onDraw(Canvas canvas) {
        Paint fillPaint;
        super.onDraw(canvas);
        Rect srcRect = new Rect(0, 0, this.silhouette.getWidth(), this.silhouette.getHeight());
        Rect destRect = new Rect(0, 0, getWidth(), getHeight());
        canvas.drawBitmap(this.silhouette, srcRect, destRect, this.gray);
        if (this.levelPercent < 20.0f) {
            fillPaint = this.red;
        } else if (this.levelPercent < 50.0f) {
            fillPaint = this.yellow;
        } else {
            fillPaint = this.green;
        }
        srcRect.set(0, 0, (int) ((((float) this.silhouette.getWidth()) * this.levelPercent) / 100.0f), this.silhouette.getHeight());
        destRect.set(0, 0, (int) ((((float) getWidth()) * this.levelPercent) / 100.0f), getHeight());
        canvas.drawBitmap(this.silhouette, srcRect, destRect, fillPaint);
        if (this.pluggedIn) {
            srcRect.set(0, 0, this.plug.getWidth(), this.plug.getHeight());
            destRect.set(0, 0, getWidth(), getHeight());
            canvas.drawBitmap(this.plug, srcRect, destRect, new Paint());
        }
    }
}
