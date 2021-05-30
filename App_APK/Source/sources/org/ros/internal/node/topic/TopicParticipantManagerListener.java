package org.ros.internal.node.topic;

public interface TopicParticipantManagerListener {
    void onPublisherAdded(DefaultPublisher<?> defaultPublisher);

    void onPublisherRemoved(DefaultPublisher<?> defaultPublisher);

    void onSubscriberAdded(DefaultSubscriber<?> defaultSubscriber);

    void onSubscriberRemoved(DefaultSubscriber<?> defaultSubscriber);
}
