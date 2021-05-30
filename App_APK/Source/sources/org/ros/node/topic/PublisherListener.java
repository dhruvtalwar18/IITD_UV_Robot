package org.ros.node.topic;

import org.ros.internal.node.RegistrantListener;
import org.ros.internal.node.topic.SubscriberIdentifier;

public interface PublisherListener<T> extends RegistrantListener<Publisher<T>> {
    void onNewSubscriber(Publisher<T> publisher, SubscriberIdentifier subscriberIdentifier);

    void onShutdown(Publisher<T> publisher);
}
