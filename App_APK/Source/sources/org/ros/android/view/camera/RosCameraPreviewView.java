package org.ros.android.view.camera;

import android.content.Context;
import android.util.AttributeSet;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;

public class RosCameraPreviewView extends CameraPreviewView implements NodeMain {
    public RosCameraPreviewView(Context context) {
        super(context);
    }

    public RosCameraPreviewView(Context context, AttributeSet attrs) {
        super(context, attrs);
    }

    public RosCameraPreviewView(Context context, AttributeSet attrs, int defStyle) {
        super(context, attrs, defStyle);
    }

    public GraphName getDefaultNodeName() {
        return GraphName.of("ros_camera_preview_view");
    }

    public void onStart(ConnectedNode connectedNode) {
        setRawImageListener(new CompressedImagePublisher(connectedNode));
    }

    public void onShutdown(Node node) {
    }

    public void onShutdownComplete(Node node) {
    }

    public void onError(Node node, Throwable throwable) {
    }
}
