package org.ros.android.view;

import android.content.Context;
import android.graphics.Point;
import android.util.AttributeSet;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.view.animation.Animation;
import android.view.animation.AnimationSet;
import android.view.animation.LinearInterpolator;
import android.view.animation.RotateAnimation;
import android.view.animation.ScaleAnimation;
import android.widget.ImageView;
import android.widget.RelativeLayout;
import android.widget.TextView;
import geometry_msgs.Twist;
import java.util.Timer;
import java.util.TimerTask;
import nav_msgs.Odometry;
import org.bytedeco.javacpp.opencv_stitching;
import org.ros.android.android_core_components.R;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

public class VirtualJoystickView extends RelativeLayout implements Animation.AnimationListener, MessageListener<Odometry>, NodeMain {
    private static final float BOX_TO_CIRCLE_RATIO = 1.363636f;
    private static final float FLOAT_EPSILON = 0.001f;
    private static final int INVALID_POINTER_ID = -1;
    private static final float ORIENTATION_TACK_FADE_RANGE = 40.0f;
    private static final float POST_LOCK_MAGNET_THETA = 20.0f;
    private static final float THUMB_DIVET_RADIUS = 16.5f;
    private static final long TURN_IN_PLACE_CONFIRMATION_DELAY = 200;
    private float contactRadius;
    private float contactTheta;
    private Point contactUpLocation;
    private volatile float currentOrientation;
    /* access modifiers changed from: private */
    public ImageView currentRotationRange;
    /* access modifiers changed from: private */
    public Twist currentVelocityCommand;
    private float deadZoneRatio;
    private boolean holonomic;
    /* access modifiers changed from: private */
    public ImageView intensity;
    private float joystickRadius;
    private ImageView lastVelocityDivet;
    private float magnetTheta;
    private boolean magnetizedXAxis;
    private TextView magnitudeText;
    private RelativeLayout mainLayout;
    private float normalizedMagnitude;
    private float normalizingMultiplier;
    private ImageView[] orientationWidget;
    private float parentSize;
    private int pointerId;
    /* access modifiers changed from: private */
    public ImageView previousRotationRange;
    private boolean previousVelocityMode;
    /* access modifiers changed from: private */
    public volatile boolean publishVelocity;
    /* access modifiers changed from: private */
    public Publisher<Twist> publisher;
    private Timer publisherTimer;
    private float rightTurnOffset;
    private ImageView thumbDivet;
    private String topicName;
    /* access modifiers changed from: private */
    public volatile boolean turnInPlaceMode;
    private float turnInPlaceStartTheta;

    public VirtualJoystickView(Context context) {
        super(context);
        this.magnetTheta = 10.0f;
        this.deadZoneRatio = Float.NaN;
        this.joystickRadius = Float.NaN;
        this.parentSize = Float.NaN;
        this.turnInPlaceStartTheta = Float.NaN;
        this.pointerId = -1;
        initVirtualJoystick(context);
        this.topicName = "~cmd_vel";
    }

    public VirtualJoystickView(Context context, AttributeSet attrs) {
        super(context, attrs);
        this.magnetTheta = 10.0f;
        this.deadZoneRatio = Float.NaN;
        this.joystickRadius = Float.NaN;
        this.parentSize = Float.NaN;
        this.turnInPlaceStartTheta = Float.NaN;
        this.pointerId = -1;
        initVirtualJoystick(context);
        this.topicName = "~cmd_vel";
    }

    public VirtualJoystickView(Context context, AttributeSet attrs, int defStyle) {
        super(context, attrs, defStyle);
        this.magnetTheta = 10.0f;
        this.deadZoneRatio = Float.NaN;
        this.joystickRadius = Float.NaN;
        this.parentSize = Float.NaN;
        this.turnInPlaceStartTheta = Float.NaN;
        this.pointerId = -1;
        this.topicName = "~cmd_vel";
    }

    public void setHolonomic(boolean enabled) {
        this.holonomic = enabled;
    }

    public void onAnimationEnd(Animation animation) {
        this.contactRadius = 0.0f;
        this.normalizedMagnitude = 0.0f;
        updateMagnitudeText();
    }

    public void onAnimationRepeat(Animation animation) {
    }

    public void onAnimationStart(Animation animation) {
    }

    public void onNewMessage(Odometry message) {
        double w = message.getPose().getPose().getOrientation().getW();
        double x = message.getPose().getPose().getOrientation().getX();
        double y = message.getPose().getPose().getOrientation().getZ();
        double z = message.getPose().getPose().getOrientation().getY();
        this.currentOrientation = (float) (-((Math.atan2(((y * 2.0d) * w) - ((2.0d * x) * z), (((x * x) - (y * y)) - (z * z)) + (w * w)) * 180.0d) / 3.141592653589793d));
        if (this.turnInPlaceMode) {
            post(new Runnable() {
                public void run() {
                    VirtualJoystickView.this.updateTurnInPlaceRotation();
                }
            });
            postInvalidate();
        }
    }

    public boolean onTouchEvent(MotionEvent event) {
        int action = event.getAction();
        int i = action & 255;
        if (i != 6) {
            switch (i) {
                case 0:
                    this.pointerId = event.getPointerId(event.getActionIndex());
                    onContactDown();
                    if (!inLastContactRange(event.getX(event.getActionIndex()), event.getY(event.getActionIndex()))) {
                        onContactMove(event.getX(event.getActionIndex()), event.getY(event.getActionIndex()));
                        break;
                    } else {
                        this.previousVelocityMode = true;
                        onContactMove(((float) this.contactUpLocation.x) + this.joystickRadius, ((float) this.contactUpLocation.y) + this.joystickRadius);
                        break;
                    }
                case 1:
                    break;
                case 2:
                    if (this.pointerId != -1) {
                        if (this.previousVelocityMode) {
                            if (!inLastContactRange(event.getX(event.getActionIndex()), event.getY(event.getActionIndex()))) {
                                this.previousVelocityMode = false;
                                break;
                            } else {
                                onContactMove(((float) this.contactUpLocation.x) + this.joystickRadius, ((float) this.contactUpLocation.y) + this.joystickRadius);
                                break;
                            }
                        } else {
                            onContactMove(event.getX(event.findPointerIndex(this.pointerId)), event.getY(event.findPointerIndex(this.pointerId)));
                            break;
                        }
                    }
                    break;
            }
        }
        if (((65280 & action) >> 8) == this.pointerId) {
            onContactUp();
        }
        return true;
    }

    public void EnableSnapping() {
        this.magnetTheta = 10.0f;
    }

    public void DisableSnapping() {
        this.magnetTheta = 1.0f;
    }

    /* access modifiers changed from: protected */
    public void onLayout(boolean changed, int l, int t, int r, int b) {
        super.onLayout(changed, l, t, r, b);
        if (this.mainLayout.getWidth() != this.mainLayout.getHeight()) {
            setOnTouchListener((View.OnTouchListener) null);
        }
        this.parentSize = (float) this.mainLayout.getWidth();
        if (this.parentSize < 200.0f || this.parentSize > 400.0f) {
            setOnTouchListener((View.OnTouchListener) null);
        }
        this.joystickRadius = (float) (this.mainLayout.getWidth() / 2);
        this.normalizingMultiplier = BOX_TO_CIRCLE_RATIO / (this.parentSize / 2.0f);
        this.deadZoneRatio = this.normalizingMultiplier * THUMB_DIVET_RADIUS;
        this.magnitudeText.setTextSize(this.parentSize / 12.0f);
    }

    private void animateIntensityCircle(float endScale) {
        AnimationSet intensityCircleAnimation = new AnimationSet(true);
        intensityCircleAnimation.setInterpolator(new LinearInterpolator());
        intensityCircleAnimation.setFillAfter(true);
        RotateAnimation rotateAnim = new RotateAnimation(this.contactTheta, this.contactTheta, this.joystickRadius, this.joystickRadius);
        rotateAnim.setInterpolator(new LinearInterpolator());
        rotateAnim.setDuration(0);
        rotateAnim.setFillAfter(true);
        intensityCircleAnimation.addAnimation(rotateAnim);
        ScaleAnimation scaleAnim = new ScaleAnimation(this.contactRadius, endScale, this.contactRadius, endScale, this.joystickRadius, this.joystickRadius);
        scaleAnim.setDuration(0);
        scaleAnim.setFillAfter(true);
        intensityCircleAnimation.addAnimation(scaleAnim);
        this.intensity.startAnimation(intensityCircleAnimation);
    }

    private void animateIntensityCircle(float endScale, long duration) {
        AnimationSet intensityCircleAnimation = new AnimationSet(true);
        intensityCircleAnimation.setInterpolator(new LinearInterpolator());
        intensityCircleAnimation.setFillAfter(true);
        intensityCircleAnimation.setAnimationListener(this);
        RotateAnimation rotateAnim = new RotateAnimation(this.contactTheta, this.contactTheta, this.joystickRadius, this.joystickRadius);
        rotateAnim.setInterpolator(new LinearInterpolator());
        rotateAnim.setDuration(duration);
        rotateAnim.setFillAfter(true);
        intensityCircleAnimation.addAnimation(rotateAnim);
        ScaleAnimation scaleAnimation = new ScaleAnimation(this.contactRadius, endScale, this.contactRadius, endScale, this.joystickRadius, this.joystickRadius);
        scaleAnimation.setDuration(duration);
        scaleAnimation.setFillAfter(true);
        intensityCircleAnimation.addAnimation(scaleAnimation);
        this.intensity.startAnimation(intensityCircleAnimation);
    }

    private void animateOrientationWidgets() {
        for (int i = 0; i < this.orientationWidget.length; i++) {
            float deltaTheta = differenceBetweenAngles((float) (i * 15), this.contactTheta);
            if (deltaTheta < ORIENTATION_TACK_FADE_RANGE) {
                this.orientationWidget[i].setAlpha(1.0f - (deltaTheta / ORIENTATION_TACK_FADE_RANGE));
            } else {
                this.orientationWidget[i].setAlpha(0.0f);
            }
        }
    }

    private float differenceBetweenAngles(float angle0, float angle1) {
        return Math.abs((((angle0 + 180.0f) - angle1) % 360.0f) - 180.0f);
    }

    private void endTurnInPlaceRotation() {
        this.turnInPlaceMode = false;
        this.currentRotationRange.setAlpha(0.0f);
        this.previousRotationRange.setAlpha(0.0f);
        this.intensity.setAlpha(1.0f);
    }

    private void initVirtualJoystick(Context context) {
        setGravity(17);
        LayoutInflater.from(context).inflate(R.layout.virtual_joystick, this, true);
        this.mainLayout = (RelativeLayout) findViewById(R.id.virtual_joystick_layout);
        this.magnitudeText = (TextView) findViewById(R.id.magnitude);
        this.intensity = (ImageView) findViewById(R.id.intensity);
        this.thumbDivet = (ImageView) findViewById(R.id.thumb_divet);
        this.orientationWidget = new ImageView[24];
        this.orientationWidget[0] = (ImageView) findViewById(R.id.widget_0_degrees);
        this.orientationWidget[1] = (ImageView) findViewById(R.id.widget_15_degrees);
        this.orientationWidget[2] = (ImageView) findViewById(R.id.widget_30_degrees);
        this.orientationWidget[3] = (ImageView) findViewById(R.id.widget_45_degrees);
        this.orientationWidget[4] = (ImageView) findViewById(R.id.widget_60_degrees);
        this.orientationWidget[5] = (ImageView) findViewById(R.id.widget_75_degrees);
        this.orientationWidget[6] = (ImageView) findViewById(R.id.widget_90_degrees);
        this.orientationWidget[7] = (ImageView) findViewById(R.id.widget_105_degrees);
        this.orientationWidget[8] = (ImageView) findViewById(R.id.widget_120_degrees);
        this.orientationWidget[9] = (ImageView) findViewById(R.id.widget_135_degrees);
        this.orientationWidget[10] = (ImageView) findViewById(R.id.widget_150_degrees);
        this.orientationWidget[11] = (ImageView) findViewById(R.id.widget_165_degrees);
        this.orientationWidget[12] = (ImageView) findViewById(R.id.widget_180_degrees);
        this.orientationWidget[13] = (ImageView) findViewById(R.id.widget_195_degrees);
        this.orientationWidget[14] = (ImageView) findViewById(R.id.widget_210_degrees);
        this.orientationWidget[15] = (ImageView) findViewById(R.id.widget_225_degrees);
        this.orientationWidget[16] = (ImageView) findViewById(R.id.widget_240_degrees);
        this.orientationWidget[17] = (ImageView) findViewById(R.id.widget_255_degrees);
        this.orientationWidget[18] = (ImageView) findViewById(R.id.widget_270_degrees);
        this.orientationWidget[19] = (ImageView) findViewById(R.id.widget_285_degrees);
        this.orientationWidget[20] = (ImageView) findViewById(R.id.widget_300_degrees);
        this.orientationWidget[21] = (ImageView) findViewById(R.id.widget_315_degrees);
        this.orientationWidget[22] = (ImageView) findViewById(R.id.widget_330_degrees);
        this.orientationWidget[23] = (ImageView) findViewById(R.id.widget_345_degrees);
        for (ImageView tack : this.orientationWidget) {
            tack.setAlpha(0.0f);
            tack.setVisibility(4);
        }
        TextView textView = this.magnitudeText;
        double d = (double) (this.contactTheta + 90.0f);
        Double.isNaN(d);
        textView.setTranslationX((float) (Math.cos((d * 3.141592653589793d) / 180.0d) * 40.0d));
        TextView textView2 = this.magnitudeText;
        double d2 = (double) (this.contactTheta + 90.0f);
        Double.isNaN(d2);
        textView2.setTranslationY((float) (Math.sin((d2 * 3.141592653589793d) / 180.0d) * 40.0d));
        animateIntensityCircle(0.0f);
        this.contactTheta = 0.0f;
        animateOrientationWidgets();
        this.currentRotationRange = (ImageView) findViewById(R.id.top_angle_slice);
        this.previousRotationRange = (ImageView) findViewById(R.id.mid_angle_slice);
        this.currentRotationRange.setAlpha(0.0f);
        this.previousRotationRange.setAlpha(0.0f);
        this.lastVelocityDivet = (ImageView) findViewById(R.id.previous_velocity_divet);
        this.contactUpLocation = new Point(0, 0);
        this.holonomic = false;
        for (ImageView tack2 : this.orientationWidget) {
            tack2.setVisibility(4);
        }
    }

    private void onContactDown() {
        this.thumbDivet.setAlpha(1.0f);
        this.magnitudeText.setAlpha(1.0f);
        this.lastVelocityDivet.setAlpha(0.0f);
        for (ImageView tack : this.orientationWidget) {
            tack.setVisibility(0);
        }
        this.publishVelocity = true;
    }

    private void onContactMove(float x, float y) {
        float thumbDivetX = x - this.joystickRadius;
        float thumbDivetY = y - this.joystickRadius;
        this.contactTheta = (float) (((Math.atan2((double) thumbDivetY, (double) thumbDivetX) * 180.0d) / 3.141592653589793d) + 90.0d);
        this.contactRadius = ((float) Math.sqrt((double) ((thumbDivetX * thumbDivetX) + (thumbDivetY * thumbDivetY)))) * this.normalizingMultiplier;
        this.normalizedMagnitude = (this.contactRadius - this.deadZoneRatio) / (1.0f - this.deadZoneRatio);
        if (this.contactRadius >= 1.0f) {
            thumbDivetX /= this.contactRadius;
            thumbDivetY /= this.contactRadius;
            this.normalizedMagnitude = 1.0f;
            this.contactRadius = 1.0f;
        } else if (this.contactRadius < this.deadZoneRatio) {
            thumbDivetX = 0.0f;
            thumbDivetY = 0.0f;
            this.normalizedMagnitude = 0.0f;
        }
        float thumbDivetX2 = thumbDivetX;
        float thumbDivetY2 = thumbDivetY;
        if (!this.magnetizedXAxis) {
            if ((this.contactTheta + 360.0f) % 90.0f < this.magnetTheta) {
                this.contactTheta -= (this.contactTheta + 360.0f) % 90.0f;
            } else if ((this.contactTheta + 360.0f) % 90.0f > 90.0f - this.magnetTheta) {
                this.contactTheta += 90.0f - ((this.contactTheta + 360.0f) % 90.0f);
            }
            if (floatCompare(this.contactTheta, 90.0f) || floatCompare(this.contactTheta, 270.0f)) {
                this.magnetizedXAxis = true;
            }
        } else if (differenceBetweenAngles((this.contactTheta + 360.0f) % 360.0f, 90.0f) < POST_LOCK_MAGNET_THETA) {
            this.contactTheta = 90.0f;
        } else if (differenceBetweenAngles((this.contactTheta + 360.0f) % 360.0f, 270.0f) < POST_LOCK_MAGNET_THETA) {
            this.contactTheta = 270.0f;
        } else {
            this.magnetizedXAxis = false;
        }
        animateIntensityCircle(this.contactRadius);
        animateOrientationWidgets();
        updateThumbDivet(thumbDivetX2, thumbDivetY2);
        updateMagnitudeText();
        if (this.holonomic) {
            double d = (double) this.normalizedMagnitude;
            double d2 = (double) this.contactTheta;
            Double.isNaN(d2);
            double cos = Math.cos((d2 * 3.141592653589793d) / 180.0d);
            Double.isNaN(d);
            double d3 = (double) this.normalizedMagnitude;
            double d4 = (double) this.contactTheta;
            Double.isNaN(d4);
            double sin = Math.sin((d4 * 3.141592653589793d) / 180.0d);
            Double.isNaN(d3);
            publishVelocity(d * cos, sin * d3, opencv_stitching.Stitcher.ORIG_RESOL);
        } else {
            double d5 = (double) this.normalizedMagnitude;
            double d6 = (double) this.contactTheta;
            Double.isNaN(d6);
            double cos2 = Math.cos((d6 * 3.141592653589793d) / 180.0d);
            Double.isNaN(d5);
            double d7 = (double) this.normalizedMagnitude;
            double d8 = (double) this.contactTheta;
            Double.isNaN(d8);
            double sin2 = Math.sin((d8 * 3.141592653589793d) / 180.0d);
            Double.isNaN(d7);
            publishVelocity(d5 * cos2, opencv_stitching.Stitcher.ORIG_RESOL, d7 * sin2);
        }
        updateTurnInPlaceMode();
    }

    private void updateTurnInPlaceMode() {
        if (!this.turnInPlaceMode) {
            if (floatCompare(this.contactTheta, 270.0f)) {
                this.turnInPlaceMode = true;
                this.rightTurnOffset = 0.0f;
            } else if (floatCompare(this.contactTheta, 90.0f)) {
                this.turnInPlaceMode = true;
                this.rightTurnOffset = 15.0f;
            } else {
                return;
            }
            initiateTurnInPlace();
            new Timer().schedule(new TimerTask() {
                public void run() {
                    VirtualJoystickView.this.post(new Runnable() {
                        public void run() {
                            if (VirtualJoystickView.this.turnInPlaceMode) {
                                VirtualJoystickView.this.currentRotationRange.setAlpha(1.0f);
                                VirtualJoystickView.this.previousRotationRange.setAlpha(1.0f);
                                VirtualJoystickView.this.intensity.setAlpha(0.2f);
                            }
                        }
                    });
                    VirtualJoystickView.this.postInvalidate();
                }
            }, TURN_IN_PLACE_CONFIRMATION_DELAY);
        } else if (!floatCompare(this.contactTheta, 270.0f) && !floatCompare(this.contactTheta, 90.0f)) {
            endTurnInPlaceRotation();
        }
    }

    private void onContactUp() {
        animateIntensityCircle(0.0f, (long) (this.normalizedMagnitude * 1000.0f));
        this.magnitudeText.setAlpha(0.4f);
        this.lastVelocityDivet.setTranslationX(this.thumbDivet.getTranslationX());
        this.lastVelocityDivet.setTranslationY(this.thumbDivet.getTranslationY());
        this.lastVelocityDivet.setAlpha(0.4f);
        this.contactUpLocation.x = (int) this.thumbDivet.getTranslationX();
        this.contactUpLocation.y = (int) this.thumbDivet.getTranslationY();
        updateThumbDivet(0.0f, 0.0f);
        this.pointerId = -1;
        publishVelocity(opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL);
        this.publishVelocity = false;
        this.publisher.publish(this.currentVelocityCommand);
        endTurnInPlaceRotation();
        for (ImageView tack : this.orientationWidget) {
            tack.setVisibility(4);
        }
    }

    private void publishVelocity(double linearVelocityX, double linearVelocityY, double angularVelocityZ) {
        this.currentVelocityCommand.getLinear().setX(linearVelocityX);
        this.currentVelocityCommand.getLinear().setY(-linearVelocityY);
        this.currentVelocityCommand.getLinear().setZ(opencv_stitching.Stitcher.ORIG_RESOL);
        this.currentVelocityCommand.getAngular().setX(opencv_stitching.Stitcher.ORIG_RESOL);
        this.currentVelocityCommand.getAngular().setY(opencv_stitching.Stitcher.ORIG_RESOL);
        this.currentVelocityCommand.getAngular().setZ(-angularVelocityZ);
    }

    private void initiateTurnInPlace() {
        this.turnInPlaceStartTheta = (this.currentOrientation + 360.0f) % 360.0f;
        RotateAnimation rotateAnim = new RotateAnimation(this.rightTurnOffset, this.rightTurnOffset, this.joystickRadius, this.joystickRadius);
        rotateAnim.setInterpolator(new LinearInterpolator());
        rotateAnim.setDuration(0);
        rotateAnim.setFillAfter(true);
        this.currentRotationRange.startAnimation(rotateAnim);
        RotateAnimation rotateAnim2 = new RotateAnimation(15.0f, 15.0f, this.joystickRadius, this.joystickRadius);
        rotateAnim2.setInterpolator(new LinearInterpolator());
        rotateAnim2.setDuration(0);
        rotateAnim2.setFillAfter(true);
        this.previousRotationRange.startAnimation(rotateAnim2);
    }

    private void updateMagnitudeText() {
        if (!this.turnInPlaceMode) {
            TextView textView = this.magnitudeText;
            textView.setText(String.valueOf((int) (this.normalizedMagnitude * 100.0f)) + "%");
            TextView textView2 = this.magnitudeText;
            double d = (double) (this.parentSize / 4.0f);
            double d2 = (double) (this.contactTheta + 90.0f);
            Double.isNaN(d2);
            double cos = Math.cos((d2 * 3.141592653589793d) / 180.0d);
            Double.isNaN(d);
            textView2.setTranslationX((float) (d * cos));
            TextView textView3 = this.magnitudeText;
            double d3 = (double) (this.parentSize / 4.0f);
            double d4 = (double) (this.contactTheta + 90.0f);
            Double.isNaN(d4);
            double sin = Math.sin((d4 * 3.141592653589793d) / 180.0d);
            Double.isNaN(d3);
            textView3.setTranslationY((float) (d3 * sin));
        }
    }

    /* access modifiers changed from: private */
    public void updateTurnInPlaceRotation() {
        float offsetTheta = 360.0f - (((this.turnInPlaceStartTheta - ((this.currentOrientation + 360.0f) % 360.0f)) + 360.0f) % 360.0f);
        this.magnitudeText.setText(String.valueOf((int) offsetTheta));
        float offsetTheta2 = (float) ((int) (offsetTheta - (offsetTheta % 15.0f)));
        RotateAnimation rotateAnim = new RotateAnimation(this.rightTurnOffset + offsetTheta2, this.rightTurnOffset + offsetTheta2, this.joystickRadius, this.joystickRadius);
        rotateAnim.setInterpolator(new LinearInterpolator());
        rotateAnim.setDuration(0);
        rotateAnim.setFillAfter(true);
        this.currentRotationRange.startAnimation(rotateAnim);
        RotateAnimation rotateAnim2 = new RotateAnimation(offsetTheta2 + 15.0f, 15.0f + offsetTheta2, this.joystickRadius, this.joystickRadius);
        rotateAnim2.setInterpolator(new LinearInterpolator());
        rotateAnim2.setDuration(0);
        rotateAnim2.setFillAfter(true);
        this.previousRotationRange.startAnimation(rotateAnim2);
    }

    private void updateThumbDivet(float x, float y) {
        this.thumbDivet.setTranslationX(-16.5f);
        this.thumbDivet.setTranslationY(-16.5f);
        this.thumbDivet.setRotation(this.contactTheta);
        this.thumbDivet.setTranslationX(x);
        this.thumbDivet.setTranslationY(y);
    }

    private boolean floatCompare(float v1, float v2) {
        if (Math.abs(v1 - v2) < FLOAT_EPSILON) {
            return true;
        }
        return false;
    }

    private boolean inLastContactRange(float x, float y) {
        if (Math.sqrt((double) ((((x - ((float) this.contactUpLocation.x)) - this.joystickRadius) * ((x - ((float) this.contactUpLocation.x)) - this.joystickRadius)) + (((y - ((float) this.contactUpLocation.y)) - this.joystickRadius) * ((y - ((float) this.contactUpLocation.y)) - this.joystickRadius)))) < 16.5d) {
            return true;
        }
        return false;
    }

    public void setTopicName(String topicName2) {
        this.topicName = topicName2;
    }

    public GraphName getDefaultNodeName() {
        return GraphName.of("android_15/virtual_joystick_view");
    }

    public void onStart(ConnectedNode connectedNode) {
        this.publisher = connectedNode.newPublisher(this.topicName, Twist._TYPE);
        this.currentVelocityCommand = this.publisher.newMessage();
        connectedNode.newSubscriber("odom", Odometry._TYPE).addMessageListener(this);
        this.publisherTimer = new Timer();
        this.publisherTimer.schedule(new TimerTask() {
            public void run() {
                if (VirtualJoystickView.this.publishVelocity) {
                    VirtualJoystickView.this.publisher.publish(VirtualJoystickView.this.currentVelocityCommand);
                }
            }
        }, 0, 80);
    }

    public void onShutdown(Node node) {
    }

    public void onShutdownComplete(Node node) {
        this.publisherTimer.cancel();
        this.publisherTimer.purge();
    }

    public void onError(Node node, Throwable throwable) {
    }
}
