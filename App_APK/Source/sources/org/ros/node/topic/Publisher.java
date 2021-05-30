package org.ros.node.topic;

import java.util.concurrent.TimeUnit;
import org.ros.internal.node.topic.TopicParticipant;

public interface Publisher<T> extends TopicParticipant {
    void addListener(PublisherListener<T> publisherListener);

    boolean getLatchMode();

    int getNumberOfSubscribers();

    boolean hasSubscribers();

    T newMessage();

    void publish(T t);

    void setLatchMode(boolean z);

    void shutdown();

    void shutdown(long j, TimeUnit timeUnit);
}
