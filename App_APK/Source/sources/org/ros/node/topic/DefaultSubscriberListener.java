package org.ros.node.topic;

import org.ros.internal.node.topic.PublisherIdentifier;

public class DefaultSubscriberListener<T> implements SubscriberListener<T> {
    public void onMasterRegistrationSuccess(Subscriber<T> subscriber) {
    }

    public void onMasterRegistrationFailure(Subscriber<T> subscriber) {
    }

    public void onMasterUnregistrationSuccess(Subscriber<T> subscriber) {
    }

    public void onMasterUnregistrationFailure(Subscriber<T> subscriber) {
    }

    public void onNewPublisher(Subscriber<T> subscriber, PublisherIdentifier publisherIdentifier) {
    }

    public void onShutdown(Subscriber<T> subscriber) {
    }
}
