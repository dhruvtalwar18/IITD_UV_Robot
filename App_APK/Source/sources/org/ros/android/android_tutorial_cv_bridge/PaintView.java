package org.ros.android.android_tutorial_cv_bridge;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.support.v4.view.InputDeviceCompat;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;
import android.widget.TextView;

public class PaintView extends View implements View.OnTouchListener {
    map1 m;
    Paint mPaint = new Paint();
    TextView mTVCoordinates = null;
    float mX = -100.0f;
    float mY = -100.0f;
    Paint p = new Paint();
    Paint p1 = new Paint();

    public PaintView(Context context, AttributeSet attributeSet) {
        super(context, attributeSet);
    }

    /* access modifiers changed from: protected */
    public void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        this.mPaint.setColor(-16711936);
        this.p.setColor(InputDeviceCompat.SOURCE_ANY);
        this.p1.setColor(-65536);
        this.m = new map1();
        canvas.drawCircle(this.mX, this.mY, 100.0f, this.p);
        canvas.drawCircle(this.mX, this.mY, 5.0f, this.mPaint);
        invalidate();
    }

    public void setTextView(TextView tv) {
        this.mTVCoordinates = tv;
    }

    public boolean onTouch(View v, MotionEvent event) {
        if (event.getAction() != 0) {
            return true;
        }
        this.mX = event.getX();
        this.mY = event.getY();
        if (this.mTVCoordinates == null) {
            return true;
        }
        TextView textView = this.mTVCoordinates;
        textView.setText("X :" + this.mX + " , Y :" + this.mY);
        return true;
    }
}
