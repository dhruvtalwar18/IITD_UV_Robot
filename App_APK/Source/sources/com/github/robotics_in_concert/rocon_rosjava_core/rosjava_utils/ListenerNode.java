package com.github.robotics_in_concert.rocon_rosjava_core.rosjava_utils;

import java.util.concurrent.TimeoutException;
import org.ros.message.MessageListener;
import org.ros.namespace.NodeNameResolver;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

public class ListenerNode<MsgType> {
    String errorMessage = "";
    private MessageListener<MsgType> listener;
    /* access modifiers changed from: private */
    public MsgType msg = null;
    private Subscriber<MsgType> subscriber;

    public void connect(ConnectedNode connectedNode, String topicName, String topicType) {
        this.msg = null;
        this.errorMessage = "";
        this.subscriber = connectedNode.newSubscriber(NodeNameResolver.newRoot().resolve(topicName).toString(), topicType);
        setupListener();
    }

    public void waitForResponse() throws ListenerException, TimeoutException {
        int count = 0;
        while (this.msg == null) {
            if (this.errorMessage == "") {
                try {
                    Thread.sleep(200);
                    if (count != 20) {
                        count++;
                    } else {
                        this.errorMessage = "timed out waiting for a " + this.subscriber.getTopicName() + "publication";
                        throw new TimeoutException(this.errorMessage);
                    }
                } catch (Exception e) {
                    throw new ListenerException((Throwable) e);
                }
            } else {
                throw new ListenerException(this.errorMessage);
            }
        }
    }

    public void waitForNextResponse() throws ListenerException, TimeoutException {
        this.errorMessage = "";
        this.msg = null;
        try {
            waitForResponse();
        } catch (ListenerException e) {
            throw e;
        } catch (TimeoutException e2) {
            throw e2;
        }
    }

    private void setupListener() {
        this.listener = new MessageListener<MsgType>() {
            public void onNewMessage(MsgType message) {
                Object unused = ListenerNode.this.msg = message;
            }
        };
        this.subscriber.addMessageListener(this.listener);
    }

    public MsgType getMessage() {
        return this.msg;
    }
}
