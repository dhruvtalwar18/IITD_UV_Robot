package com.github.rosjava.android_remocons.common_tools.layouts;

import android.content.Context;
import android.util.AttributeSet;
import android.widget.Checkable;
import android.widget.LinearLayout;

public class CheckableLinearLayout extends LinearLayout implements Checkable {
    private static final int[] CHECKED_STATE_SET = {16842912};
    private boolean checked = false;

    public CheckableLinearLayout(Context ctx) {
        super(ctx);
    }

    public CheckableLinearLayout(Context context, AttributeSet attrs) {
        super(context, attrs);
    }

    public boolean isChecked() {
        return this.checked;
    }

    public void setChecked(boolean checked2) {
        if (this.checked != checked2) {
            this.checked = checked2;
            refreshDrawableState();
        }
    }

    public void toggle() {
        setChecked(!this.checked);
    }

    /* access modifiers changed from: protected */
    public int[] onCreateDrawableState(int extraSpace) {
        int[] drawableState = super.onCreateDrawableState(extraSpace + 1);
        if (isChecked()) {
            mergeDrawableStates(drawableState, CHECKED_STATE_SET);
        }
        return drawableState;
    }
}
