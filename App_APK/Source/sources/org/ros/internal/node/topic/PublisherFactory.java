package org.ros.internal.node.topic;

import java.util.concurrent.ScheduledExecutorService;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.message.MessageFactory;
import org.ros.message.MessageSerializer;
import org.ros.namespace.GraphName;
import org.ros.node.topic.DefaultPublisherListener;
import org.ros.node.topic.Publisher;

public class PublisherFactory {
    private final ScheduledExecutorService executorService;
    private final MessageFactory messageFactory;
    private final Object mutex = new Object();
    private final NodeIdentifier nodeIdentifier;
    /* access modifiers changed from: private */
    public final TopicParticipantManager topicParticipantManager;

    public PublisherFactory(NodeIdentifier nodeIdentifier2, TopicParticipantManager topicParticipantManager2, MessageFactory messageFactory2, ScheduledExecutorService executorService2) {
        this.nodeIdentifier = nodeIdentifier2;
        this.topicParticipantManager = topicParticipantManager2;
        this.messageFactory = messageFactory2;
        this.executorService = executorService2;
    }

    public <T> Publisher<T> newOrExisting(TopicDeclaration topicDeclaration, MessageSerializer<T> messageSerializer) {
        GraphName topicName = topicDeclaration.getName();
        synchronized (this.mutex) {
            if (this.topicParticipantManager.hasPublisher(topicName)) {
                DefaultPublisher<?> publisher = this.topicParticipantManager.getPublisher(topicName);
                return publisher;
            }
            DefaultPublisher defaultPublisher = new DefaultPublisher(this.nodeIdentifier, topicDeclaration, messageSerializer, this.messageFactory, this.executorService);
            defaultPublisher.addListener(new DefaultPublisherListener<T>() {
                public void onNewSubscriber(Publisher<T> publisher, SubscriberIdentifier subscriberIdentifier) {
                    PublisherFactory.this.topicParticipantManager.addPublisherConnection((DefaultPublisher) publisher, subscriberIdentifier);
                }

                public void onShutdown(Publisher<T> publisher) {
                    PublisherFactory.this.topicParticipantManager.removePublisher((DefaultPublisher) publisher);
                }
            });
            this.topicParticipantManager.addPublisher(defaultPublisher);
            return defaultPublisher;
        }
    }
}
