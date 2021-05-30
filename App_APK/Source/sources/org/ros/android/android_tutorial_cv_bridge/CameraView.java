package org.ros.android.android_tutorial_cv_bridge;

import org.apache.commons.logging.Log;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import sensor_msgs.CompressedImage;
import sensor_msgs.Image;

public class CameraView extends AbstractNodeMain {
    static ChannelBuffer rosCameraPreviewView;

    public GraphName getDefaultNodeName() {
        return GraphName.of("rosjava_tutorial_pubsub/camera");
    }

    public void onStart(ConnectedNode connectedNode) {
        final Log log = connectedNode.getLog();
        connectedNode.newSubscriber("camera", CompressedImage._TYPE).addMessageListener(new MessageListener<Image>() {
            public void onNewMessage(Image message) {
                CameraView.rosCameraPreviewView = message.getData();
                Log log = log;
                log.info("I heardjjjjjjnjjnknjkkbkjbjbj: \"" + CameraView.rosCameraPreviewView + "\"");
            }
        });
    }

    public ChannelBuffer getLevel() {
        return rosCameraPreviewView;
    }
}
