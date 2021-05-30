package org.ros.android.view;

import android.content.Context;
import android.content.SharedPreferences;
import android.util.AttributeSet;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.widget.ImageView;
import android.widget.RelativeLayout;
import org.ros.android.android_core_components.R;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import sensor_msgs.JointState;

public class PanTiltView extends RelativeLayout implements View.OnTouchListener, NodeMain {
    private static final float GUIDE_LENGTH = 149.0f;
    private static final String HOME_PAN_KEY_NAME = "HOME_PAN";
    private static final String HOME_TILT_KEY_NAME = "HOME_TILT";
    private static final int INVALID_POINTER_ID = -1;
    private static final int INVALID_POINTER_LOCATION = -1;
    private static final String MAX_PAN_KEY_NAME = "MAX_PAN";
    private static final int MAX_TACK_COORDINATE = 184;
    private static final String MAX_TILT_KEY_NAME = "MAX_TILT";
    private static final int MIDDLE_AREA = 0;
    private static final String MIN_PAN_KEY_NAME = "MIN_PAN";
    private static final int MIN_TACK_COORDINATE = 35;
    private static final String MIN_TILT_KEY_NAME = "MIN_TILT";
    private static final int RIGHT_AREA = 2;
    private static final String SHARED_PREFERENCE_NAME = "PAN_TILT_VIEW_PREFERENCE";
    private static final int TOP_AREA = 1;
    private ImageView desiredTack;
    private ImageView homeIcon;
    private float homePan;
    private float homeTilt;
    private int initialPointerLocation;
    private RelativeLayout mainLayout;
    private float maxPan;
    private float maxTilt;
    private float minPan;
    private float minTilt;
    private int pointerId;
    private Publisher<JointState> publisher;
    private ImageView[] rightLargeTack;
    private ImageView[] rightSmallTack;
    private ImageView[] topLargeTack;
    private ImageView[] topSmallTack;
    private ImageView[] zoomLitBar;
    private int zoomValue;

    public PanTiltView(Context context) {
        this(context, (AttributeSet) null, 0);
    }

    public PanTiltView(Context context, AttributeSet attrs) {
        this(context, attrs, 0);
    }

    public PanTiltView(Context context, AttributeSet attrs, int defStyle) {
        super(context, attrs, defStyle);
        this.minPan = -1.0f;
        this.maxPan = 1.0f;
        this.minTilt = -1.0f;
        this.maxTilt = 1.0f;
        this.homePan = 0.0f;
        this.homeTilt = 0.0f;
        this.pointerId = -1;
        this.zoomValue = 0;
        LayoutInflater.from(context).inflate(R.layout.pan_tilt, this, true);
        loadSettings();
        initPanTiltWidget();
    }

    public boolean onTouch(View v, MotionEvent event) {
        int action = event.getAction() & 255;
        if (action != 6) {
            switch (action) {
                case 0:
                    this.pointerId = event.getPointerId(event.getActionIndex());
                    onContactDown(event.getX(event.getActionIndex()), event.getY(event.getActionIndex()));
                    return true;
                case 1:
                    break;
                case 2:
                    if (this.pointerId == -1) {
                        return true;
                    }
                    onContactMove(event.getX(event.findPointerIndex(this.pointerId)), event.getY(event.findPointerIndex(this.pointerId)));
                    return true;
                default:
                    return true;
            }
        }
        this.pointerId = -1;
        this.initialPointerLocation = -1;
        return true;
    }

    private void onContactMove(float x, float y) {
        if (this.initialPointerLocation == 0) {
            updateTopTack(x);
            updateRightTack(y);
        } else if (this.initialPointerLocation == 1) {
            updateTopTack(x);
        } else if (this.initialPointerLocation == 2) {
            updateRightTack(y);
        } else if (x < 75.0f && y > 120.0f && y < 248.0f) {
            if (y < 21.0f + 120.0f) {
                this.zoomValue = 5;
            } else if (y < (2.0f * 21.0f) + 120.0f) {
                this.zoomValue = 4;
            } else if (y < (3.0f * 21.0f) + 120.0f) {
                this.zoomValue = 3;
            } else if (y < (4.0f * 21.0f) + 120.0f) {
                this.zoomValue = 2;
            } else if (y < (5.0f * 21.0f) + 120.0f) {
                this.zoomValue = 1;
            } else if (y < (6.0f * 21.0f) + 120.0f) {
                this.zoomValue = 0;
            }
            updateZoomBars();
        }
    }

    private void onContactDown(float x, float y) {
        if (x > 75.0f && x < 357.0f && y > 50.0f && y < 278.0f) {
            this.initialPointerLocation = 0;
            updateTopTack(x);
            updateRightTack(y);
        } else if (y < 40.0f && x > 75.0f && x < 357.0f) {
            this.initialPointerLocation = 1;
            updateTopTack(x);
        } else if (x > 361.0f && y > 45.0f && y < 366.0f) {
            this.initialPointerLocation = 2;
            updateRightTack(y);
        } else if (x < 75.0f && y > 55.0f && y < 120.0f) {
            this.zoomValue++;
            if (this.zoomValue > 5) {
                this.zoomValue = 5;
            }
            updateZoomBars();
        } else if (x < 75.0f && y > 248.0f) {
            this.zoomValue--;
            if (this.zoomValue < 0) {
                this.zoomValue = 0;
            }
            updateZoomBars();
        } else if (x < 75.0f && y > 120.0f && y < 248.0f) {
            if (y < 21.0f + 120.0f) {
                this.zoomValue = 5;
            } else if (y < (2.0f * 21.0f) + 120.0f) {
                this.zoomValue = 4;
            } else if (y < (3.0f * 21.0f) + 120.0f) {
                this.zoomValue = 3;
            } else if (y < (4.0f * 21.0f) + 120.0f) {
                this.zoomValue = 2;
            } else if (y < (5.0f * 21.0f) + 120.0f) {
                this.zoomValue = 1;
            } else if (y < (6.0f * 21.0f) + 120.0f) {
                this.zoomValue = 0;
            }
            updateZoomBars();
        }
    }

    private void updateZoomBars() {
        for (int i = 0; i < this.zoomLitBar.length; i++) {
            this.zoomLitBar[4 - i].setVisibility(4);
        }
        for (int i2 = 0; i2 < this.zoomValue; i2++) {
            this.zoomLitBar[4 - i2].setVisibility(0);
        }
    }

    private void updateRightTack(float y) {
        float offset = (float) (this.desiredTack.getHeight() / 2);
        if (y < offset + 40.0f) {
            y = offset + 40.0f;
        } else if (y > 278.0f - offset) {
            y = 278.0f - offset;
        } else if (y < this.homeIcon.getTranslationY() + ((float) (this.homeIcon.getHeight() / 5)) + ((float) (getHeight() / 2)) && y > (this.homeIcon.getTranslationY() + ((float) (getHeight() / 2))) - ((float) (this.homeIcon.getHeight() / 5))) {
            y = this.homeIcon.getTranslationY() + ((float) (getHeight() / 2));
        }
        this.desiredTack.setTranslationY(y - ((float) (this.mainLayout.getHeight() / 2)));
        publishTilt(y);
        for (int i = 0; i < this.rightLargeTack.length; i++) {
            if (Math.abs((y - ((float) (this.mainLayout.getHeight() / 2))) - this.rightLargeTack[i].getTranslationY()) < 12.0f) {
                this.rightLargeTack[i].setAlpha(1.0f);
            } else {
                this.rightLargeTack[i].setAlpha(0.0f);
            }
        }
        for (int i2 = 0; i2 < this.rightSmallTack.length; i2++) {
            if (Math.abs((y - ((float) (this.mainLayout.getHeight() / 2))) - this.rightSmallTack[i2].getTranslationY()) < 50.0f) {
                this.rightSmallTack[i2].setAlpha(1.0f - (Math.abs((y - ((float) (this.mainLayout.getHeight() / 2))) - this.rightSmallTack[i2].getTranslationY()) / 50.0f));
            } else {
                this.rightSmallTack[i2].setAlpha(0.0f);
            }
        }
    }

    private void updateTopTack(float x) {
        float offset = (float) (this.desiredTack.getWidth() / 2);
        if (x < offset + 75.0f) {
            x = offset + 75.0f;
        } else if (x > 357.0f - offset) {
            x = 357.0f - offset;
        } else if (x < this.homeIcon.getTranslationX() + ((float) (this.homeIcon.getWidth() / 5)) + ((float) (getWidth() / 2)) && x > (this.homeIcon.getTranslationX() + ((float) (getWidth() / 2))) - ((float) (this.homeIcon.getWidth() / 5))) {
            x = this.homeIcon.getTranslationX() + ((float) (getWidth() / 2));
        }
        this.desiredTack.setTranslationX(x - ((float) (this.mainLayout.getWidth() / 2)));
        publishPan(x);
        for (int i = 0; i < this.topLargeTack.length; i++) {
            if (Math.abs((x - ((float) (this.mainLayout.getWidth() / 2))) - this.topLargeTack[i].getTranslationX()) < 13.0f) {
                this.topLargeTack[i].setAlpha(1.0f);
            } else {
                this.topLargeTack[i].setAlpha(0.0f);
            }
        }
        for (int i2 = 0; i2 < this.topSmallTack.length; i2++) {
            if (Math.abs((x - ((float) (this.mainLayout.getWidth() / 2))) - this.topSmallTack[i2].getTranslationX()) < 50.0f) {
                this.topSmallTack[i2].setAlpha(1.0f - (Math.abs((x - ((float) (this.mainLayout.getWidth() / 2))) - this.topSmallTack[i2].getTranslationX()) / 50.0f));
            } else {
                this.topSmallTack[i2].setAlpha(0.0f);
            }
        }
    }

    private void initPanTiltWidget() {
        this.mainLayout = (RelativeLayout) findViewById(R.id.pan_tilt_layout);
        this.desiredTack = (ImageView) findViewById(R.id.pt_divet);
        this.topLargeTack = new ImageView[10];
        this.topSmallTack = new ImageView[10];
        this.rightLargeTack = new ImageView[7];
        this.rightSmallTack = new ImageView[7];
        for (int i = 0; i < this.topLargeTack.length; i++) {
            this.topLargeTack[i] = new ImageView(getContext());
            this.topSmallTack[i] = new ImageView(getContext());
        }
        this.topLargeTack[0] = (ImageView) findViewById(R.id.pan_large_marker_0);
        this.topLargeTack[1] = (ImageView) findViewById(R.id.pan_large_marker_1);
        this.topLargeTack[2] = (ImageView) findViewById(R.id.pan_large_marker_2);
        this.topLargeTack[3] = (ImageView) findViewById(R.id.pan_large_marker_3);
        this.topLargeTack[4] = (ImageView) findViewById(R.id.pan_large_marker_4);
        this.topLargeTack[5] = (ImageView) findViewById(R.id.pan_large_marker_5);
        this.topLargeTack[6] = (ImageView) findViewById(R.id.pan_large_marker_6);
        this.topLargeTack[7] = (ImageView) findViewById(R.id.pan_large_marker_7);
        this.topLargeTack[8] = (ImageView) findViewById(R.id.pan_large_marker_8);
        this.topLargeTack[9] = (ImageView) findViewById(R.id.pan_large_marker_9);
        this.topSmallTack[0] = (ImageView) findViewById(R.id.pan_small_marker_0);
        this.topSmallTack[1] = (ImageView) findViewById(R.id.pan_small_marker_1);
        this.topSmallTack[2] = (ImageView) findViewById(R.id.pan_small_marker_2);
        this.topSmallTack[3] = (ImageView) findViewById(R.id.pan_small_marker_3);
        this.topSmallTack[4] = (ImageView) findViewById(R.id.pan_small_marker_4);
        this.topSmallTack[5] = (ImageView) findViewById(R.id.pan_small_marker_5);
        this.topSmallTack[6] = (ImageView) findViewById(R.id.pan_small_marker_6);
        this.topSmallTack[7] = (ImageView) findViewById(R.id.pan_small_marker_7);
        this.topSmallTack[8] = (ImageView) findViewById(R.id.pan_small_marker_8);
        this.topSmallTack[9] = (ImageView) findViewById(R.id.pan_small_marker_9);
        for (int i2 = 0; i2 < this.topLargeTack.length; i2++) {
            this.topLargeTack[i2].setAlpha(0.0f);
            this.topSmallTack[i2].setAlpha(0.0f);
        }
        for (int i3 = 0; i3 < this.rightLargeTack.length; i3++) {
            this.rightLargeTack[i3] = new ImageView(getContext());
            this.rightSmallTack[i3] = new ImageView(getContext());
        }
        this.rightLargeTack[0] = (ImageView) findViewById(R.id.tilt_large_marker_0);
        this.rightLargeTack[1] = (ImageView) findViewById(R.id.tilt_large_marker_1);
        this.rightLargeTack[2] = (ImageView) findViewById(R.id.tilt_large_marker_2);
        this.rightLargeTack[3] = (ImageView) findViewById(R.id.tilt_large_marker_3);
        this.rightLargeTack[4] = (ImageView) findViewById(R.id.tilt_large_marker_4);
        this.rightLargeTack[5] = (ImageView) findViewById(R.id.tilt_large_marker_5);
        this.rightLargeTack[6] = (ImageView) findViewById(R.id.tilt_large_marker_6);
        this.rightSmallTack[0] = (ImageView) findViewById(R.id.tilt_small_marker_0);
        this.rightSmallTack[1] = (ImageView) findViewById(R.id.tilt_small_marker_1);
        this.rightSmallTack[2] = (ImageView) findViewById(R.id.tilt_small_marker_2);
        this.rightSmallTack[3] = (ImageView) findViewById(R.id.tilt_small_marker_3);
        this.rightSmallTack[4] = (ImageView) findViewById(R.id.tilt_small_marker_4);
        this.rightSmallTack[5] = (ImageView) findViewById(R.id.tilt_small_marker_5);
        this.rightSmallTack[6] = (ImageView) findViewById(R.id.tilt_small_marker_6);
        for (int i4 = 0; i4 < this.rightLargeTack.length; i4++) {
            this.rightLargeTack[i4].setAlpha(0.0f);
            this.rightSmallTack[i4].setAlpha(0.0f);
        }
        this.zoomLitBar = new ImageView[5];
        this.zoomLitBar[0] = (ImageView) findViewById(R.id.zoom_bar_lit_0);
        this.zoomLitBar[1] = (ImageView) findViewById(R.id.zoom_bar_lit_1);
        this.zoomLitBar[2] = (ImageView) findViewById(R.id.zoom_bar_lit_2);
        this.zoomLitBar[3] = (ImageView) findViewById(R.id.zoom_bar_lit_3);
        this.zoomLitBar[4] = (ImageView) findViewById(R.id.zoom_bar_lit_4);
        this.homeIcon = (ImageView) findViewById(R.id.pt_home_marker);
    }

    private void loadSettings() {
        SharedPreferences settings = getContext().getSharedPreferences(SHARED_PREFERENCE_NAME, 0);
        settings.getFloat(MAX_PAN_KEY_NAME, this.maxPan);
        settings.getFloat(MIN_PAN_KEY_NAME, this.minPan);
        settings.getFloat(MAX_TILT_KEY_NAME, this.maxTilt);
        settings.getFloat(MIN_TILT_KEY_NAME, this.minTilt);
        settings.getFloat(HOME_PAN_KEY_NAME, this.homePan);
        settings.getFloat(HOME_TILT_KEY_NAME, this.homeTilt);
    }

    private void publishPan(float x) {
        float pan = ((this.maxPan - this.minPan) * (1.0f - ((184.0f - x) / GUIDE_LENGTH))) + this.minPan;
        JointState jointState = this.publisher.newMessage();
        jointState.getName().add("pan");
        jointState.setPosition(new double[]{(double) pan});
        this.publisher.publish(jointState);
    }

    private void publishTilt(float y) {
        float tilt = ((this.maxTilt - this.minTilt) * (1.0f - ((184.0f - y) / GUIDE_LENGTH))) + this.minTilt;
        JointState jointState = this.publisher.newMessage();
        jointState.getName().add("tilt");
        jointState.setPosition(new double[]{(double) tilt});
        this.publisher.publish(jointState);
    }

    public GraphName getDefaultNodeName() {
        return GraphName.of("android_15/pan_tilt_view");
    }

    public void onStart(ConnectedNode connectedNode) {
        this.publisher = connectedNode.newPublisher("ptu_cmd", JointState._TYPE);
    }

    public void onShutdown(Node node) {
    }

    public void onShutdownComplete(Node node) {
    }

    public void onError(Node node, Throwable throwable) {
    }
}
