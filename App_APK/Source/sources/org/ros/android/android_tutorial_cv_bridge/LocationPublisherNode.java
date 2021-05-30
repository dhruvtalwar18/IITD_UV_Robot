package org.ros.android.android_tutorial_cv_bridge;

import android.location.Location;
import android.location.LocationListener;
import android.os.Bundle;
import android.util.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import sensor_msgs.NavSatFix;
import std_msgs.Header;

public class LocationPublisherNode extends AbstractNodeMain {
    /* access modifiers changed from: private */
    public static final String TAG = LocationPublisherNode.class.getSimpleName();
    /* access modifiers changed from: private */
    public static float maxElapse = (1000.0f / minFrequency);
    private static float maxFrequency = 100.0f;
    /* access modifiers changed from: private */
    public static float minElapse = (1000.0f / maxFrequency);
    private static float minFrequency = 20.0f;
    /* access modifiers changed from: private */
    public Location cachedLocation;
    /* access modifiers changed from: private */
    public boolean isMessagePending = false;
    private OnFrameIdChangeListener locationFrameIdChangeListener = new OnFrameIdChangeListener() {
        public void onFrameIdChanged(String newFrameId) {
            String unused = LocationPublisherNode.this.navSatFixFrameId = newFrameId;
        }
    };
    private final LocationListener locationListener = new LocationListener() {
        public void onLocationChanged(Location location) {
            if (location != null) {
                Location unused = LocationPublisherNode.this.cachedLocation = location;
                boolean unused2 = LocationPublisherNode.this.isMessagePending = true;
            }
        }

        public void onStatusChanged(String s, int i, Bundle bundle) {
            String access$200 = LocationPublisherNode.TAG;
            Log.d(access$200, "Provider: " + s + ", Status: " + i + ", Extras: " + bundle);
        }

        public void onProviderEnabled(String s) {
            String access$200 = LocationPublisherNode.TAG;
            Log.d(access$200, "Provider enabled: " + s);
        }

        public void onProviderDisabled(String s) {
            String access$200 = LocationPublisherNode.TAG;
            Log.d(access$200, "Provider disabled: " + s);
        }
    };
    /* access modifiers changed from: private */
    public String navSatFixFrameId;
    /* access modifiers changed from: private */
    public long previousPublishTime = System.currentTimeMillis();
    private String topic_name = "fix";

    public void onStart(final ConnectedNode connectedNode) {
        final Publisher<NavSatFix> locationPublisher = connectedNode.newPublisher(this.topic_name, NavSatFix._TYPE);
        final NavSatFix navSatFix = locationPublisher.newMessage();
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            Header header = ((Header) connectedNode.getTopicMessageFactory().newFromType(Header._TYPE));
            int sequenceNumber = 1;

            /* access modifiers changed from: protected */
            public void loop() throws InterruptedException {
                if (LocationPublisherNode.this.cachedLocation == null || (!LocationPublisherNode.this.isMessagePending && ((float) (System.currentTimeMillis() - LocationPublisherNode.this.previousPublishTime)) < LocationPublisherNode.maxElapse)) {
                    Thread.sleep(1);
                    return;
                }
                this.header.setStamp(connectedNode.getCurrentTime());
                this.header.setFrameId(LocationPublisherNode.this.navSatFixFrameId);
                this.header.setSeq(this.sequenceNumber);
                navSatFix.setHeader(this.header);
                navSatFix.setLatitude(LocationPublisherNode.this.cachedLocation.getLatitude());
                navSatFix.setLongitude(LocationPublisherNode.this.cachedLocation.getLongitude());
                Log.d(LocationPublisherNode.TAG, "LOCATION PUBLISHED");
                locationPublisher.publish(navSatFix);
                long remainingTime = (long) (LocationPublisherNode.minElapse - ((float) (System.currentTimeMillis() - LocationPublisherNode.this.previousPublishTime)));
                if (remainingTime > 0) {
                    Thread.sleep(remainingTime);
                }
                long unused = LocationPublisherNode.this.previousPublishTime = System.currentTimeMillis();
                boolean unused2 = LocationPublisherNode.this.isMessagePending = false;
                this.sequenceNumber++;
            }
        });
    }

    public GraphName getDefaultNodeName() {
        return GraphName.of("ros_android_sensors/location_publisher_node");
    }

    public LocationListener getLocationListener() {
        return this.locationListener;
    }

    public OnFrameIdChangeListener getFrameIdListener() {
        return this.locationFrameIdChangeListener;
    }
}
