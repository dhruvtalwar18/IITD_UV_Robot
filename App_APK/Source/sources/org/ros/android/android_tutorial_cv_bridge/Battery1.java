package org.ros.android.android_tutorial_cv_bridge;

import android.widget.EditText;
import org.apache.commons.logging.Log;
import org.ros.android.tutorial_CompressedImage_cv_bridge.R;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import std_msgs.Int32;

public class Battery1 extends AbstractNodeMain {
    static int i;
    /* access modifiers changed from: private */
    public MainActivityCompressedJavacv mainActivityCompressedJavacv;

    public Battery1(MainActivityCompressedJavacv mainActivityCompressedJavacv2) {
        this.mainActivityCompressedJavacv = mainActivityCompressedJavacv2;
    }

    public GraphName getDefaultNodeName() {
        return GraphName.of("rosjava_tutorial_pubsub/battery");
    }

    public void onStart(ConnectedNode connectedNode) {
        super.onStart(connectedNode);
        final Log log = connectedNode.getLog();
        connectedNode.newSubscriber("battery", Int32._TYPE).addMessageListener(new MessageListener<Int32>() {
            public void onNewMessage(Int32 message) {
                Battery1.this.mainActivityCompressedJavacv.runOnUiThread(new Runnable() {
                    public void run() {
                        ((EditText) Battery1.this.mainActivityCompressedJavacv.findViewById(R.id.battery_level)).setText("Battery = " + Battery1.i + "%");
                    }
                });
                Battery1.i = message.getData();
                Log log = log;
                log.info("I heard: \"" + Battery1.i + "\"");
            }
        });
    }

    public int getLevel() {
        return i;
    }
}
