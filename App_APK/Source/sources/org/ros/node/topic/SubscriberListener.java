package org.ros.node.topic;

import org.ros.internal.node.RegistrantListener;
import org.ros.internal.node.topic.PublisherIdentifier;

public interface SubscriberListener<T> extends RegistrantListener<Subscriber<T>> {
    void onNewPublisher(Subscriber<T> subscriber, PublisherIdentifier publisherIdentifier);

    void onShutdown(Subscriber<T> subscriber);
}
