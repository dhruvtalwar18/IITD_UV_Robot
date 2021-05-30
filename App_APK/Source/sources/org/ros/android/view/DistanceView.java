package org.ros.android.view;

import android.content.Context;
import android.opengl.GLSurfaceView;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;
import geometry_msgs.Twist;
import java.util.ArrayList;
import org.bytedeco.javacpp.opencv_stitching;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import sensor_msgs.LaserScan;

public class DistanceView extends GLSurfaceView implements View.OnTouchListener, NodeMain, MessageListener<LaserScan> {
    private double contactDistance;
    /* access modifiers changed from: private */
    public DistanceRenderer distanceRenderer;
    private String laserTopic;
    private double normalizedZoomValue;

    public DistanceView(Context context) {
        this(context, (AttributeSet) null);
    }

    public DistanceView(Context context, AttributeSet attrs) {
        super(context, attrs);
        this.distanceRenderer = new DistanceRenderer();
        setEGLConfigChooser(8, 8, 8, 8, 16, 0);
        getHolder().setFormat(-3);
        setZOrderOnTop(true);
        setRenderer(this.distanceRenderer);
        setRenderMode(0);
    }

    public void setTopicName(String topicName) {
        this.laserTopic = topicName;
    }

    public GraphName getDefaultNodeName() {
        return GraphName.of("android_15/distance_view");
    }

    public void onStart(ConnectedNode connectedNode) {
        connectedNode.newSubscriber(this.laserTopic, LaserScan._TYPE).addMessageListener(this);
        connectedNode.newSubscriber("cmd_vel", Twist._TYPE).addMessageListener(new MessageListener<Twist>() {
            public void onNewMessage(final Twist robotVelocity) {
                DistanceView.this.post(new Runnable() {
                    public void run() {
                        DistanceView.this.distanceRenderer.currentSpeed(robotVelocity.getLinear().getX());
                    }
                });
            }
        });
        setOnTouchListener(this);
        this.distanceRenderer.loadPreferences(getContext());
    }

    public void onShutdown(Node node) {
    }

    public void onShutdownComplete(Node node) {
        this.distanceRenderer.savePreferences(getContext());
    }

    public void onError(Node node, Throwable throwable) {
    }

    public void onNewMessage(final LaserScan message) {
        queueEvent(new Runnable() {
            public void run() {
                ArrayList arrayList = new ArrayList();
                float minDistToObject = message.getRangeMax();
                float[] ranges = message.getRanges();
                int length = ranges.length;
                float minDistToObject2 = minDistToObject;
                for (int i = 0; i < length; i++) {
                    float range = ranges[i];
                    arrayList.add(Float.valueOf(range));
                    minDistToObject2 = minDistToObject2 > range ? range : minDistToObject2;
                }
                DistanceView.this.distanceRenderer.updateRange(arrayList, message.getRangeMax(), message.getRangeMin(), message.getAngleMin(), message.getAngleIncrement(), minDistToObject2);
                DistanceView.this.requestRender();
            }
        });
    }

    public void setZoomMode(ZoomMode mode) {
        this.distanceRenderer.setZoomMode(mode);
    }

    public void lockZoom() {
        this.distanceRenderer.lockZoom();
    }

    public void unlockZoom() {
        this.distanceRenderer.unlockZoom();
    }

    public void currentSpeed(double speed) {
        this.distanceRenderer.currentSpeed(speed);
    }

    public boolean onTouch(View v, MotionEvent event) {
        int action = event.getAction() & 255;
        if (action != 2) {
            if (action == 5) {
                this.contactDistance = calculateDistance(event.getX(0), event.getY(0), event.getX(1), event.getY(1));
            }
        } else if (event.getPointerCount() == 2) {
            double currentContactDistance = calculateDistance(event.getX(0), event.getY(0), event.getX(1), event.getY(1));
            double d = this.normalizedZoomValue;
            double width = (double) (getWidth() / 2);
            Double.isNaN(width);
            this.normalizedZoomValue = d + ((currentContactDistance - this.contactDistance) / width);
            if (this.normalizedZoomValue > 1.0d) {
                this.normalizedZoomValue = 1.0d;
            } else if (this.normalizedZoomValue < opencv_stitching.Stitcher.ORIG_RESOL) {
                this.normalizedZoomValue = opencv_stitching.Stitcher.ORIG_RESOL;
            }
            this.distanceRenderer.setNormalizedZoom((float) this.normalizedZoomValue);
            this.contactDistance = currentContactDistance;
        }
        return true;
    }

    private double calculateDistance(float x1, float y1, float x2, float y2) {
        return Math.sqrt((double) (((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1))));
    }
}
