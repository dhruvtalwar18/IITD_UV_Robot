package org.ros.node.topic;

import org.ros.internal.node.topic.SubscriberIdentifier;

public class DefaultPublisherListener<T> implements PublisherListener<T> {
    public void onMasterRegistrationSuccess(Publisher<T> publisher) {
    }

    public void onMasterRegistrationFailure(Publisher<T> publisher) {
    }

    public void onMasterUnregistrationSuccess(Publisher<T> publisher) {
    }

    public void onMasterUnregistrationFailure(Publisher<T> publisher) {
    }

    public void onNewSubscriber(Publisher<T> publisher, SubscriberIdentifier subscriberIdentifier) {
    }

    public void onShutdown(Publisher<T> publisher) {
    }
}
