package org.ros.android.android_tutorial_cv_bridge;

import android.util.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import std_msgs.Float32;

public class autonomous extends AbstractNodeMain {
    /* access modifiers changed from: private */
    public MainActivity mainActivity;

    public autonomous(MainActivity mainActivity2) {
        this.mainActivity = mainActivity2;
    }

    public GraphName getDefaultNodeName() {
        Log.d(Emergency_Stop.class.getName(), "aaaaadcdcdcdscbsdbcjsjcbjdsjcjasnxkaksxnkaskxnksxksd");
        return GraphName.of("rosjava_tutorial_pubsub/autonomous");
    }

    public void onStart(ConnectedNode connectedNode) {
        final Publisher<Float32> publisher = connectedNode.newPublisher("autonomous", Float32._TYPE);
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            private int sequenceNumber;

            /* access modifiers changed from: protected */
            public void setup() {
            }

            /* access modifiers changed from: protected */
            public void loop() throws InterruptedException {
                Float32 str = (Float32) publisher.newMessage();
                if (autonomous.this.mainActivity.autonomous_status == 1) {
                    str.setData(1.0f);
                    publisher.publish(str);
                    autonomous.this.mainActivity.autonomous_status = 0;
                } else {
                    str.setData(0.0f);
                    publisher.publish(str);
                }
                Thread.currentThread();
                Thread.sleep(100);
            }
        });
    }
}
