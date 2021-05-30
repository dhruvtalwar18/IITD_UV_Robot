package org.ros.android.android_tutorial_cv_bridge;

import android.widget.EditText;
import org.apache.commons.logging.Log;
import org.ros.android.tutorial_CompressedImage_cv_bridge.R;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import std_msgs.String;

public class status extends AbstractNodeMain {
    static String i;
    /* access modifiers changed from: private */
    public MainActivity mainActivity;

    public status(MainActivity mainActivity2) {
        this.mainActivity = mainActivity2;
    }

    public GraphName getDefaultNodeName() {
        return GraphName.of("rosjava_tutorial_pubsub/robot_status");
    }

    public void onStart(ConnectedNode connectedNode) {
        super.onStart(connectedNode);
        final Log log = connectedNode.getLog();
        log.info("I heard: \"pppppppppkkkkkkkkk\"");
        connectedNode.newSubscriber("robot_status", String._TYPE).addMessageListener(new MessageListener<String>() {
            public void onNewMessage(String message) {
                status.this.mainActivity.runOnUiThread(new Runnable() {
                    public void run() {
                        ((EditText) status.this.mainActivity.findViewById(R.id.status)).setText("Robot Status = " + status.i + "");
                    }
                });
                status.i = message.getData();
                Log log = log;
                log.info("I heard: \"" + status.i + "kkkkkkkkk\"");
            }
        });
    }

    public String getLevel() {
        return i;
    }
}
