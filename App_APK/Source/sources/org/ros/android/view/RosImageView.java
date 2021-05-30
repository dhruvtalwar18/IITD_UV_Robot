package org.ros.android.view;

import android.content.Context;
import android.graphics.Bitmap;
import android.util.AttributeSet;
import android.widget.ImageView;
import org.ros.android.MessageCallable;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;

public class RosImageView<T> extends ImageView implements NodeMain {
    /* access modifiers changed from: private */
    public MessageCallable<Bitmap, T> callable;
    private String messageType;
    private String topicName;

    public RosImageView(Context context) {
        super(context);
    }

    public RosImageView(Context context, AttributeSet attrs) {
        super(context, attrs);
    }

    public RosImageView(Context context, AttributeSet attrs, int defStyle) {
        super(context, attrs, defStyle);
    }

    public void setTopicName(String topicName2) {
        this.topicName = topicName2;
    }

    public void setMessageType(String messageType2) {
        this.messageType = messageType2;
    }

    public void setMessageToBitmapCallable(MessageCallable<Bitmap, T> callable2) {
        this.callable = callable2;
    }

    public GraphName getDefaultNodeName() {
        return GraphName.of("ros_image_view");
    }

    public void onStart(ConnectedNode connectedNode) {
        connectedNode.newSubscriber(this.topicName, this.messageType).addMessageListener(new MessageListener<T>() {
            public void onNewMessage(final T message) {
                RosImageView.this.post(new Runnable() {
                    public void run() {
                        RosImageView.this.setImageBitmap((Bitmap) RosImageView.this.callable.call(message));
                    }
                });
                RosImageView.this.postInvalidate();
            }
        });
    }

    public void onShutdown(Node node) {
    }

    public void onShutdownComplete(Node node) {
    }

    public void onError(Node node, Throwable throwable) {
    }
}
