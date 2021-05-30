package org.ros.internal.node.topic;

import java.util.concurrent.ScheduledExecutorService;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.message.MessageDeserializer;
import org.ros.namespace.GraphName;
import org.ros.node.topic.DefaultSubscriberListener;
import org.ros.node.topic.Subscriber;

public class SubscriberFactory {
    private final ScheduledExecutorService executorService;
    private final Object mutex = new Object();
    private final NodeIdentifier nodeIdentifier;
    /* access modifiers changed from: private */
    public final TopicParticipantManager topicParticipantManager;

    public SubscriberFactory(NodeIdentifier nodeIdentifier2, TopicParticipantManager topicParticipantManager2, ScheduledExecutorService executorService2) {
        this.nodeIdentifier = nodeIdentifier2;
        this.topicParticipantManager = topicParticipantManager2;
        this.executorService = executorService2;
    }

    public <T> Subscriber<T> newOrExisting(TopicDeclaration topicDeclaration, MessageDeserializer<T> messageDeserializer) {
        synchronized (this.mutex) {
            GraphName topicName = topicDeclaration.getName();
            if (this.topicParticipantManager.hasSubscriber(topicName)) {
                DefaultSubscriber<?> subscriber = this.topicParticipantManager.getSubscriber(topicName);
                return subscriber;
            }
            DefaultSubscriber<T> subscriber2 = DefaultSubscriber.newDefault(this.nodeIdentifier, topicDeclaration, this.executorService, messageDeserializer);
            subscriber2.addSubscriberListener(new DefaultSubscriberListener<T>() {
                public void onNewPublisher(Subscriber<T> subscriber, PublisherIdentifier publisherIdentifier) {
                    SubscriberFactory.this.topicParticipantManager.addSubscriberConnection((DefaultSubscriber) subscriber, publisherIdentifier);
                }

                public void onShutdown(Subscriber<T> subscriber) {
                    SubscriberFactory.this.topicParticipantManager.removeSubscriber((DefaultSubscriber) subscriber);
                }
            });
            this.topicParticipantManager.addSubscriber(subscriber2);
            return subscriber2;
        }
    }
}
