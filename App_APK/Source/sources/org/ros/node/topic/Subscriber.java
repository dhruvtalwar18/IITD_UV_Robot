package org.ros.node.topic;

import java.util.concurrent.TimeUnit;
import org.ros.internal.node.topic.TopicParticipant;
import org.ros.message.MessageListener;

public interface Subscriber<T> extends TopicParticipant {
    public static final String TOPIC_MESSAGE_TYPE_WILDCARD = "*";

    void addMessageListener(MessageListener<T> messageListener);

    void addMessageListener(MessageListener<T> messageListener, int i);

    void addSubscriberListener(SubscriberListener<T> subscriberListener);

    boolean getLatchMode();

    void removeAllMessageListeners();

    boolean removeMessageListener(MessageListener<T> messageListener);

    void shutdown();

    void shutdown(long j, TimeUnit timeUnit);
}
