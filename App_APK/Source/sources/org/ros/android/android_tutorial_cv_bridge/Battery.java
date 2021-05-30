package org.ros.android.android_tutorial_cv_bridge;

import android.widget.EditText;
import org.apache.commons.logging.Log;
import org.ros.android.tutorial_CompressedImage_cv_bridge.R;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import std_msgs.Int32;

public class Battery extends AbstractNodeMain {
    static int i;
    /* access modifiers changed from: private */
    public MainActivity mainActivity;

    public Battery(MainActivity mainActivity2) {
        this.mainActivity = mainActivity2;
    }

    public GraphName getDefaultNodeName() {
        return GraphName.of("rosjava_tutorial_pubsub/battery");
    }

    public void onStart(ConnectedNode connectedNode) {
        super.onStart(connectedNode);
        final Log log = connectedNode.getLog();
        connectedNode.newSubscriber("battery", Int32._TYPE).addMessageListener(new MessageListener<Int32>() {
            public void onNewMessage(Int32 message) {
                Battery.this.mainActivity.runOnUiThread(new Runnable() {
                    public void run() {
                        ((EditText) Battery.this.mainActivity.findViewById(R.id.et_location_frame_id)).setText("Battery = " + Battery.i + "%");
                    }
                });
                Battery.i = message.getData();
                Log log = log;
                log.info("I heard: \"" + Battery.i + "\"");
            }
        });
    }

    public int getLevel() {
        return i;
    }
}
