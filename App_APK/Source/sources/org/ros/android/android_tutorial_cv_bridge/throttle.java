package org.ros.android.android_tutorial_cv_bridge;

import android.util.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import std_msgs.Float32;

public class throttle extends AbstractNodeMain {
    /* access modifiers changed from: private */
    public MainActivityCompressedJavacv mainActivityCompressedJavacv;

    public throttle(MainActivityCompressedJavacv mainActivityCompressedJavacv2) {
        this.mainActivityCompressedJavacv = mainActivityCompressedJavacv2;
    }

    public GraphName getDefaultNodeName() {
        Log.d(Emergency_Stop.class.getName(), "aaaaadcdcdcdscbsdbcjsjcbjdsjcjasnxkaksxnkaskxnksxksd");
        return GraphName.of("rosjava_tutorial_pubsub/throttle");
    }

    public void onStart(ConnectedNode connectedNode) {
        final Publisher<Float32> publisher = connectedNode.newPublisher("throttle", Float32._TYPE);
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            private int sequenceNumber;

            /* access modifiers changed from: protected */
            public void loop() throws InterruptedException {
                Float32 str = (Float32) publisher.newMessage();
                if (throttle.this.mainActivityCompressedJavacv.status == 1) {
                    str.setData(1.0f);
                    publisher.publish(str);
                } else if (throttle.this.mainActivityCompressedJavacv.status == 2) {
                    str.setData(2.0f);
                    publisher.publish(str);
                } else if (throttle.this.mainActivityCompressedJavacv.status == 3) {
                    str.setData(3.0f);
                    publisher.publish(str);
                } else if (throttle.this.mainActivityCompressedJavacv.status == 4) {
                    str.setData(4.0f);
                    publisher.publish(str);
                } else if (throttle.this.mainActivityCompressedJavacv.status == 0) {
                    str.setData(0.0f);
                    publisher.publish(str);
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
