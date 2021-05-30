package org.ros.android.android_tutorial_cv_bridge;

import android.widget.EditText;
import org.apache.commons.logging.Log;
import org.ros.android.tutorial_CompressedImage_cv_bridge.R;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import std_msgs.Float32;

public class work_completion extends AbstractNodeMain {
    static float i;
    /* access modifiers changed from: private */
    public MainActivity mainActivity;

    public work_completion(MainActivity mainActivity2) {
        this.mainActivity = mainActivity2;
    }

    public GraphName getDefaultNodeName() {
        return GraphName.of("rosjava_tutorial_pubsub/work_comp");
    }

    public void onStart(ConnectedNode connectedNode) {
        super.onStart(connectedNode);
        final Log log = connectedNode.getLog();
        connectedNode.newSubscriber("work_comp", Float32._TYPE).addMessageListener(new MessageListener<Float32>() {
            public void onNewMessage(Float32 message) {
                work_completion.this.mainActivity.runOnUiThread(new Runnable() {
                    public void run() {
                        ((EditText) work_completion.this.mainActivity.findViewById(R.id.work)).setText("Sanitation Completed = " + work_completion.i + "%");
                    }
                });
                work_completion.i = message.getData();
                Log log = log;
                log.info("I heard: \"" + work_completion.i + "\"");
            }
        });
    }

    public float getLevel() {
        return i;
    }
}
