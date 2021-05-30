package org.ros.internal.node.topic;

import com.google.common.annotations.VisibleForTesting;
import com.google.common.collect.Sets;
import java.net.InetSocketAddress;
import java.util.Collection;
import java.util.Set;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.concurrent.ListenerGroup;
import org.ros.concurrent.SignalRunnable;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.internal.transport.ProtocolNames;
import org.ros.internal.transport.queue.IncomingMessageQueue;
import org.ros.internal.transport.tcp.TcpClientManager;
import org.ros.message.MessageDeserializer;
import org.ros.message.MessageListener;
import org.ros.node.topic.DefaultSubscriberListener;
import org.ros.node.topic.Subscriber;
import org.ros.node.topic.SubscriberListener;

public class DefaultSubscriber<T> extends DefaultTopicParticipant implements Subscriber<T> {
    private static final int DEFAULT_SHUTDOWN_TIMEOUT = 5;
    private static final TimeUnit DEFAULT_SHUTDOWN_TIMEOUT_UNITS = TimeUnit.SECONDS;
    /* access modifiers changed from: private */
    public static final Log log = LogFactory.getLog(DefaultSubscriber.class);
    private final ScheduledExecutorService executorService;
    private final IncomingMessageQueue<T> incomingMessageQueue;
    private final Set<PublisherIdentifier> knownPublishers = Sets.newHashSet();
    private final Object mutex;
    private final NodeIdentifier nodeIdentifier;
    private final ListenerGroup<SubscriberListener<T>> subscriberListeners;
    private final TcpClientManager tcpClientManager;

    public static <S> DefaultSubscriber<S> newDefault(NodeIdentifier nodeIdentifier2, TopicDeclaration description, ScheduledExecutorService executorService2, MessageDeserializer<S> deserializer) {
        return new DefaultSubscriber<>(nodeIdentifier2, description, deserializer, executorService2);
    }

    private DefaultSubscriber(NodeIdentifier nodeIdentifier2, TopicDeclaration topicDeclaration, MessageDeserializer<T> deserializer, ScheduledExecutorService executorService2) {
        super(topicDeclaration);
        this.nodeIdentifier = nodeIdentifier2;
        this.executorService = executorService2;
        this.incomingMessageQueue = new IncomingMessageQueue<>(deserializer, executorService2);
        this.tcpClientManager = new TcpClientManager(executorService2);
        this.mutex = new Object();
        this.tcpClientManager.addNamedChannelHandler(new SubscriberHandshakeHandler<>(toDeclaration().toConnectionHeader(), this.incomingMessageQueue, executorService2));
        this.subscriberListeners = new ListenerGroup<>(executorService2);
        this.subscriberListeners.add(new DefaultSubscriberListener<T>() {
            public void onMasterRegistrationSuccess(Subscriber<T> subscriber) {
                Log access$000 = DefaultSubscriber.log;
                access$000.info("Subscriber registered: " + DefaultSubscriber.this);
            }

            public void onMasterRegistrationFailure(Subscriber<T> subscriber) {
                Log access$000 = DefaultSubscriber.log;
                access$000.info("Subscriber registration failed: " + DefaultSubscriber.this);
            }

            public void onMasterUnregistrationSuccess(Subscriber<T> subscriber) {
                Log access$000 = DefaultSubscriber.log;
                access$000.info("Subscriber unregistered: " + DefaultSubscriber.this);
            }

            public void onMasterUnregistrationFailure(Subscriber<T> subscriber) {
                Log access$000 = DefaultSubscriber.log;
                access$000.info("Subscriber unregistration failed: " + DefaultSubscriber.this);
            }
        });
    }

    public SubscriberIdentifier toIdentifier() {
        return new SubscriberIdentifier(this.nodeIdentifier, getTopicDeclaration().getIdentifier());
    }

    public SubscriberDeclaration toDeclaration() {
        return new SubscriberDeclaration(toIdentifier(), getTopicDeclaration());
    }

    public Collection<String> getSupportedProtocols() {
        return ProtocolNames.SUPPORTED;
    }

    public boolean getLatchMode() {
        return this.incomingMessageQueue.getLatchMode();
    }

    public void addMessageListener(MessageListener<T> messageListener, int limit) {
        this.incomingMessageQueue.addListener(messageListener, limit);
    }

    public void addMessageListener(MessageListener<T> messageListener) {
        addMessageListener(messageListener, 1);
    }

    public boolean removeMessageListener(MessageListener<T> messageListener) {
        return this.incomingMessageQueue.removeListener(messageListener);
    }

    public void removeAllMessageListeners() {
        this.incomingMessageQueue.removeAllListeners();
    }

    @VisibleForTesting
    public void addPublisher(PublisherIdentifier publisherIdentifier, InetSocketAddress address) {
        synchronized (this.mutex) {
            if (!this.knownPublishers.contains(publisherIdentifier)) {
                this.tcpClientManager.connect(toString(), address);
                this.knownPublishers.add(publisherIdentifier);
                signalOnNewPublisher(publisherIdentifier);
            }
        }
    }

    public void updatePublishers(Collection<PublisherIdentifier> publisherIdentifiers) {
        for (PublisherIdentifier publisherIdentifier : publisherIdentifiers) {
            this.executorService.execute(new UpdatePublisherRunnable(this, this.nodeIdentifier, publisherIdentifier));
        }
    }

    public void shutdown(long timeout, TimeUnit unit) {
        signalOnShutdown(timeout, unit);
        this.incomingMessageQueue.shutdown();
        this.tcpClientManager.shutdown();
        this.subscriberListeners.shutdown();
    }

    public void shutdown() {
        shutdown(5, DEFAULT_SHUTDOWN_TIMEOUT_UNITS);
    }

    public void addSubscriberListener(SubscriberListener<T> listener) {
        this.subscriberListeners.add(listener);
    }

    public void signalOnMasterRegistrationSuccess() {
        this.subscriberListeners.signal(new SignalRunnable<SubscriberListener<T>>() {
            public void run(SubscriberListener<T> listener) {
                listener.onMasterRegistrationSuccess(this);
            }
        });
    }

    public void signalOnMasterRegistrationFailure() {
        this.subscriberListeners.signal(new SignalRunnable<SubscriberListener<T>>() {
            public void run(SubscriberListener<T> listener) {
                listener.onMasterRegistrationFailure(this);
            }
        });
    }

    public void signalOnMasterUnregistrationSuccess() {
        this.subscriberListeners.signal(new SignalRunnable<SubscriberListener<T>>() {
            public void run(SubscriberListener<T> listener) {
                listener.onMasterUnregistrationSuccess(this);
            }
        });
    }

    public void signalOnMasterUnregistrationFailure() {
        this.subscriberListeners.signal(new SignalRunnable<SubscriberListener<T>>() {
            public void run(SubscriberListener<T> listener) {
                listener.onMasterUnregistrationFailure(this);
            }
        });
    }

    public void signalOnNewPublisher(final PublisherIdentifier publisherIdentifier) {
        this.subscriberListeners.signal(new SignalRunnable<SubscriberListener<T>>() {
            public void run(SubscriberListener<T> listener) {
                listener.onNewPublisher(this, publisherIdentifier);
            }
        });
    }

    private void signalOnShutdown(long timeout, TimeUnit unit) {
        try {
            this.subscriberListeners.signal(new SignalRunnable<SubscriberListener<T>>() {
                public void run(SubscriberListener<T> listener) {
                    listener.onShutdown(this);
                }
            }, timeout, unit);
        } catch (InterruptedException e) {
        }
    }

    public String toString() {
        return "Subscriber<" + getTopicDeclaration() + ">";
    }
}
