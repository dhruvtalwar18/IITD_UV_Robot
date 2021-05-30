package org.ros.android.android_tutorial_cv_bridge;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import sensor_msgs.Imu;
import std_msgs.Header;

public class ImuPublisherNode extends AbstractNodeMain {
    private static float maxFrequency = 100.0f;
    /* access modifiers changed from: private */
    public float aPitch;
    /* access modifiers changed from: private */
    public float aRoll;
    /* access modifiers changed from: private */
    public float aYaw;
    private SensorEventListener accelerometerListener = new SensorEventListener() {
        public void onSensorChanged(SensorEvent sensorEvent) {
            if (ImuPublisherNode.this.ax != sensorEvent.values[0] || ImuPublisherNode.this.ay != sensorEvent.values[1] || ImuPublisherNode.this.az != sensorEvent.values[2]) {
                float unused = ImuPublisherNode.this.ax = sensorEvent.values[0];
                float unused2 = ImuPublisherNode.this.ay = sensorEvent.values[1];
                float unused3 = ImuPublisherNode.this.az = sensorEvent.values[2];
                boolean unused4 = ImuPublisherNode.this.isAccelerometerMessagePending = true;
            }
        }

        public void onAccuracyChanged(Sensor sensor, int i) {
        }
    };
    /* access modifiers changed from: private */
    public float ax;
    /* access modifiers changed from: private */
    public float ay;
    /* access modifiers changed from: private */
    public float az;
    private SensorEventListener gyroscopeListener = new SensorEventListener() {
        public void onSensorChanged(SensorEvent sensorEvent) {
            if (ImuPublisherNode.this.aRoll != (-sensorEvent.values[1]) || ImuPublisherNode.this.aPitch != (-sensorEvent.values[2]) || ImuPublisherNode.this.aYaw != sensorEvent.values[0]) {
                float unused = ImuPublisherNode.this.aRoll = -sensorEvent.values[1];
                float unused2 = ImuPublisherNode.this.aPitch = -sensorEvent.values[2];
                float unused3 = ImuPublisherNode.this.aYaw = sensorEvent.values[0];
                boolean unused4 = ImuPublisherNode.this.isGyroscopeMessagePending = true;
            }
        }

        public void onAccuracyChanged(Sensor sensor, int i) {
        }
    };
    /* access modifiers changed from: private */
    public String imuFrameId;
    private OnFrameIdChangeListener imuFrameIdChangeListener = new OnFrameIdChangeListener() {
        public void onFrameIdChanged(String newFrameId) {
            String unused = ImuPublisherNode.this.imuFrameId = newFrameId;
        }
    };
    /* access modifiers changed from: private */
    public boolean isAccelerometerMessagePending = false;
    /* access modifiers changed from: private */
    public boolean isGyroscopeMessagePending = false;
    /* access modifiers changed from: private */
    public boolean isOrientationMessagePending = false;
    /* access modifiers changed from: private */
    public float minElapse = (1000.0f / maxFrequency);
    private SensorEventListener orientationListener = new SensorEventListener() {
        public void onSensorChanged(SensorEvent sensorEvent) {
            if (ImuPublisherNode.this.roll != (-sensorEvent.values[1]) || ImuPublisherNode.this.pitch != (-sensorEvent.values[2]) || ImuPublisherNode.this.yaw != 360.0f - sensorEvent.values[0]) {
                float unused = ImuPublisherNode.this.roll = -sensorEvent.values[1];
                float unused2 = ImuPublisherNode.this.pitch = -sensorEvent.values[2];
                float unused3 = ImuPublisherNode.this.yaw = 360.0f - sensorEvent.values[0];
                boolean unused4 = ImuPublisherNode.this.isOrientationMessagePending = true;
            }
        }

        public void onAccuracyChanged(Sensor sensor, int i) {
        }
    };
    /* access modifiers changed from: private */
    public float pitch;
    /* access modifiers changed from: private */
    public float prevPitch;
    /* access modifiers changed from: private */
    public float prevRoll;
    /* access modifiers changed from: private */
    public float prevYaw;
    /* access modifiers changed from: private */
    public long previousPublishTime = System.currentTimeMillis();
    /* access modifiers changed from: private */
    public float roll;
    private String topic_name = "imu_data";
    /* access modifiers changed from: private */
    public float yaw;

    public GraphName getDefaultNodeName() {
        return GraphName.of("ros_android_sensors/imu_publisher_node");
    }

    public void onStart(final ConnectedNode connectedNode) {
        final Publisher<Imu> imuPublisher = connectedNode.newPublisher(this.topic_name, Imu._TYPE);
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            Header header = ((Header) connectedNode.getTopicMessageFactory().newFromType(Header._TYPE));
            Imu imuMessage = ((Imu) imuPublisher.newMessage());
            int sequenceNumber = 1;

            /* access modifiers changed from: protected */
            public void loop() throws InterruptedException {
                long currentTimeMillis = System.currentTimeMillis();
                if (!ImuPublisherNode.this.isAccelerometerMessagePending || !ImuPublisherNode.this.isGyroscopeMessagePending || !ImuPublisherNode.this.isOrientationMessagePending) {
                    Thread.sleep(1);
                    return;
                }
                this.header.setStamp(connectedNode.getCurrentTime());
                this.header.setFrameId(ImuPublisherNode.this.imuFrameId);
                this.header.setSeq(this.sequenceNumber);
                this.imuMessage.setHeader(this.header);
                this.imuMessage.getLinearAcceleration().setX((double) ImuPublisherNode.this.ax);
                this.imuMessage.getLinearAcceleration().setY((double) ImuPublisherNode.this.ay);
                this.imuMessage.getLinearAcceleration().setZ((double) ImuPublisherNode.this.az);
                float dt = ((float) (currentTimeMillis - ImuPublisherNode.this.previousPublishTime)) / 1000.0f;
                float dRoll = ImuPublisherNode.this.roll - ImuPublisherNode.this.prevRoll;
                if (dRoll > 180.0f) {
                    dRoll = 360.0f - dRoll;
                }
                float dPitch = ImuPublisherNode.this.pitch - ImuPublisherNode.this.prevPitch;
                if (dPitch > 180.0f) {
                    dPitch = 360.0f - dPitch;
                }
                float dYaw = ImuPublisherNode.this.yaw - ImuPublisherNode.this.prevYaw;
                if (dYaw > 180.0f) {
                    dYaw = 360.0f - dYaw;
                }
                this.imuMessage.getAngularVelocity().setX((double) (dRoll / dt));
                this.imuMessage.getAngularVelocity().setY((double) (dPitch / dt));
                this.imuMessage.getAngularVelocity().setZ((double) (dYaw / dt));
                float unused = ImuPublisherNode.this.prevRoll = ImuPublisherNode.this.roll;
                float unused2 = ImuPublisherNode.this.prevPitch = ImuPublisherNode.this.pitch;
                float unused3 = ImuPublisherNode.this.prevYaw = ImuPublisherNode.this.yaw;
                this.imuMessage.getOrientation().setW((double) ImuPublisherNode.this.roll);
                this.imuMessage.getOrientation().setX((double) ImuPublisherNode.this.roll);
                this.imuMessage.getOrientation().setY((double) ImuPublisherNode.this.pitch);
                this.imuMessage.getOrientation().setZ((double) ImuPublisherNode.this.yaw);
                imuPublisher.publish(this.imuMessage);
                long remainingTime = (long) (ImuPublisherNode.this.minElapse - ((float) (currentTimeMillis - ImuPublisherNode.this.previousPublishTime)));
                if (remainingTime > 0) {
                    Thread.sleep(remainingTime);
                }
                long unused4 = ImuPublisherNode.this.previousPublishTime = System.currentTimeMillis();
                boolean unused5 = ImuPublisherNode.this.isAccelerometerMessagePending = false;
                boolean unused6 = ImuPublisherNode.this.isGyroscopeMessagePending = false;
                boolean unused7 = ImuPublisherNode.this.isOrientationMessagePending = false;
                this.sequenceNumber++;
            }
        });
    }

    public SensorEventListener getAccelerometerListener() {
        return this.accelerometerListener;
    }

    public SensorEventListener getGyroscopeListener() {
        return this.gyroscopeListener;
    }

    public SensorEventListener getOrientationListener() {
        return this.orientationListener;
    }

    public OnFrameIdChangeListener getFrameIdListener() {
        return this.imuFrameIdChangeListener;
    }
}
