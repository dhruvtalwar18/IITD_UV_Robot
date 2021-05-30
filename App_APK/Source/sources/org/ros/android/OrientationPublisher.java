package org.ros.android;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import geometry_msgs.PoseStamped;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

public class OrientationPublisher extends AbstractNodeMain {
    private OrientationListener orientationListener;
    private final SensorManager sensorManager;

    private final class OrientationListener implements SensorEventListener {
        private final Publisher<PoseStamped> publisher;

        private OrientationListener(Publisher<PoseStamped> publisher2) {
            this.publisher = publisher2;
        }

        public void onAccuracyChanged(Sensor sensor, int accuracy) {
        }

        public void onSensorChanged(SensorEvent event) {
            if (event.sensor.getType() == 11) {
                float[] quaternion = new float[4];
                SensorManager.getQuaternionFromVector(quaternion, event.values);
                PoseStamped pose = this.publisher.newMessage();
                pose.getHeader().setFrameId("/map");
                pose.getHeader().setStamp(Time.fromMillis(System.currentTimeMillis()));
                pose.getPose().getOrientation().setW((double) quaternion[0]);
                pose.getPose().getOrientation().setX((double) quaternion[1]);
                pose.getPose().getOrientation().setY((double) quaternion[2]);
                pose.getPose().getOrientation().setZ((double) quaternion[3]);
                this.publisher.publish(pose);
            }
        }
    }

    public OrientationPublisher(SensorManager sensorManager2) {
        this.sensorManager = sensorManager2;
    }

    public GraphName getDefaultNodeName() {
        return GraphName.of("android/orientiation_sensor");
    }

    public void onStart(ConnectedNode connectedNode) {
        try {
            this.orientationListener = new OrientationListener(connectedNode.newPublisher("android/orientation", PoseStamped._TYPE));
            this.sensorManager.registerListener(this.orientationListener, this.sensorManager.getDefaultSensor(11), 500000);
        } catch (Exception e) {
            connectedNode.getLog().fatal(e);
        }
    }
}
