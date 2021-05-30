package org.ros.android.android_tutorial_cv_bridge;

import android.util.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import std_msgs.Float32;

public class points_node extends AbstractNodeMain {
    /* access modifiers changed from: private */
    public map1 map1;

    public points_node(map1 map12) {
        this.map1 = map12;
    }

    public GraphName getDefaultNodeName() {
        Log.d(Emergency_Stop.class.getName(), "aaaaadcdcdcdscbsdbcjsjcbjdsjcjasnxkaksxnkaskxnksxksd");
        return GraphName.of("rosjava_tutorial_pubsub/points_node");
    }

    public void onStart(ConnectedNode connectedNode) {
        final Publisher<Float32> publisherx = connectedNode.newPublisher("x_nodes", Float32._TYPE);
        final Publisher<Float32> publishery = connectedNode.newPublisher("y_nodes", Float32._TYPE);
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            private int sequenceNumber;

            /* access modifiers changed from: protected */
            public void loop() throws InterruptedException {
                if (points_node.this.map1.status == 1) {
                    Float32 strx = (Float32) publisherx.newMessage();
                    Float32 stry = (Float32) publishery.newMessage();
                    strx.setData((float) points_node.this.map1.x);
                    stry.setData((float) points_node.this.map1.y);
                    publisherx.publish(strx);
                    publishery.publish(stry);
                    points_node.this.map1.status = 0;
                }
            }
        });
    }
}
