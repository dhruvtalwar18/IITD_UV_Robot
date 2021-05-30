package org.ros.android.view;

import android.content.Context;
import android.util.AttributeSet;
import android.widget.TextView;
import org.ros.android.MessageCallable;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;

public class RosTextView<T> extends TextView implements NodeMain {
    /* access modifiers changed from: private */
    public MessageCallable<String, T> callable;
    private String messageType;
    private String topicName;

    public RosTextView(Context context) {
        super(context);
    }

    public RosTextView(Context context, AttributeSet attrs) {
        super(context, attrs);
    }

    public RosTextView(Context context, AttributeSet attrs, int defStyle) {
        super(context, attrs, defStyle);
    }

    public void setTopicName(String topicName2) {
        this.topicName = topicName2;
    }

    public void setMessageType(String messageType2) {
        this.messageType = messageType2;
    }

    public void setMessageToStringCallable(MessageCallable<String, T> callable2) {
        this.callable = callable2;
    }

    public GraphName getDefaultNodeName() {
        return GraphName.of("android_gingerbread/ros_text_view");
    }

    public void onStart(ConnectedNode connectedNode) {
        connectedNode.newSubscriber(this.topicName, this.messageType).addMessageListener(new MessageListener<T>() {
            public void onNewMessage(final T message) {
                if (RosTextView.this.callable != null) {
                    RosTextView.this.post(new Runnable() {
                        public void run() {
                            RosTextView.this.setText((CharSequence) RosTextView.this.callable.call(message));
                        }
                    });
                } else {
                    RosTextView.this.post(new Runnable() {
                        public void run() {
                            RosTextView.this.setText(message.toString());
                        }
                    });
                }
                RosTextView.this.postInvalidate();
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
