package org.ros.internal.node.topic;

import com.google.common.base.Preconditions;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.channel.Channel;
import org.ros.concurrent.ListenerGroup;
import org.ros.concurrent.SignalRunnable;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.internal.transport.ConnectionHeader;
import org.ros.internal.transport.ConnectionHeaderFields;
import org.ros.internal.transport.queue.OutgoingMessageQueue;
import org.ros.message.MessageFactory;
import org.ros.message.MessageSerializer;
import org.ros.node.topic.DefaultPublisherListener;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.PublisherListener;

public class DefaultPublisher<T> extends DefaultTopicParticipant implements Publisher<T> {
    private static final boolean DEBUG = false;
    private static final long DEFAULT_SHUTDOWN_TIMEOUT = 5;
    private static final TimeUnit DEFAULT_SHUTDOWN_TIMEOUT_UNITS = TimeUnit.SECONDS;
    /* access modifiers changed from: private */
    public static final Log log = LogFactory.getLog(DefaultPublisher.class);
    private final ListenerGroup<PublisherListener<T>> listeners;
    private final MessageFactory messageFactory;
    private final NodeIdentifier nodeIdentifier;
    private final OutgoingMessageQueue<T> outgoingMessageQueue;

    public DefaultPublisher(NodeIdentifier nodeIdentifier2, TopicDeclaration topicDeclaration, MessageSerializer<T> serializer, MessageFactory messageFactory2, ScheduledExecutorService executorService) {
        super(topicDeclaration);
        this.nodeIdentifier = nodeIdentifier2;
        this.messageFactory = messageFactory2;
        this.outgoingMessageQueue = new OutgoingMessageQueue<>(serializer, executorService);
        this.listeners = new ListenerGroup<>(executorService);
        this.listeners.add(new DefaultPublisherListener<T>() {
            public void onMasterRegistrationSuccess(Publisher<T> publisher) {
                Log access$000 = DefaultPublisher.log;
                access$000.info("Publisher registered: " + DefaultPublisher.this);
            }

            public void onMasterRegistrationFailure(Publisher<T> publisher) {
                Log access$000 = DefaultPublisher.log;
                access$000.info("Publisher registration failed: " + DefaultPublisher.this);
            }

            public void onMasterUnregistrationSuccess(Publisher<T> publisher) {
                Log access$000 = DefaultPublisher.log;
                access$000.info("Publisher unregistered: " + DefaultPublisher.this);
            }

            public void onMasterUnregistrationFailure(Publisher<T> publisher) {
                Log access$000 = DefaultPublisher.log;
                access$000.info("Publisher unregistration failed: " + DefaultPublisher.this);
            }
        });
    }

    public void setLatchMode(boolean enabled) {
        this.outgoingMessageQueue.setLatchMode(enabled);
    }

    public boolean getLatchMode() {
        return this.outgoingMessageQueue.getLatchMode();
    }

    public void shutdown(long timeout, TimeUnit unit) {
        signalOnShutdown(timeout, unit);
        this.outgoingMessageQueue.shutdown();
        this.listeners.shutdown();
    }

    public void shutdown() {
        shutdown(5, DEFAULT_SHUTDOWN_TIMEOUT_UNITS);
    }

    public PublisherIdentifier getIdentifier() {
        return new PublisherIdentifier(this.nodeIdentifier, getTopicDeclaration().getIdentifier());
    }

    public PublisherDeclaration toDeclaration() {
        return PublisherDeclaration.newFromNodeIdentifier(this.nodeIdentifier, getTopicDeclaration());
    }

    public boolean hasSubscribers() {
        return this.outgoingMessageQueue.getNumberOfChannels() > 0;
    }

    public int getNumberOfSubscribers() {
        return this.outgoingMessageQueue.getNumberOfChannels();
    }

    public T newMessage() {
        return this.messageFactory.newFromType(getTopicDeclaration().getMessageType());
    }

    public void publish(T message) {
        this.outgoingMessageQueue.add(message);
    }

    public ChannelBuffer finishHandshake(ConnectionHeader incomingHeader) {
        ConnectionHeader topicDefinitionHeader = getTopicDeclarationHeader();
        String incomingType = incomingHeader.getField(ConnectionHeaderFields.TYPE);
        String expectedType = topicDefinitionHeader.getField(ConnectionHeaderFields.TYPE);
        boolean checksumMatches = false;
        boolean messageTypeMatches = incomingType.equals(expectedType) || incomingType.equals("*");
        Preconditions.checkState(messageTypeMatches, "Unexpected message type " + incomingType + " != " + expectedType);
        String incomingChecksum = incomingHeader.getField(ConnectionHeaderFields.MD5_CHECKSUM);
        String expectedChecksum = topicDefinitionHeader.getField(ConnectionHeaderFields.MD5_CHECKSUM);
        if (incomingChecksum.equals(expectedChecksum) || incomingChecksum.equals("*")) {
            checksumMatches = true;
        }
        Preconditions.checkState(checksumMatches, "Unexpected message MD5 " + incomingChecksum + " != " + expectedChecksum);
        ConnectionHeader outgoingConnectionHeader = toDeclaration().toConnectionHeader();
        outgoingConnectionHeader.addField(ConnectionHeaderFields.LATCHING, getLatchMode() ? "1" : "0");
        return outgoingConnectionHeader.encode();
    }

    public void addSubscriber(SubscriberIdentifier subscriberIdentifer, Channel channel) {
        this.outgoingMessageQueue.addChannel(channel);
        signalOnNewSubscriber(subscriberIdentifer);
    }

    public void addListener(PublisherListener<T> listener) {
        this.listeners.add(listener);
    }

    public void signalOnMasterRegistrationSuccess() {
        this.listeners.signal(new SignalRunnable<PublisherListener<T>>() {
            public void run(PublisherListener<T> listener) {
                listener.onMasterRegistrationSuccess(this);
            }
        });
    }

    public void signalOnMasterRegistrationFailure() {
        this.listeners.signal(new SignalRunnable<PublisherListener<T>>() {
            public void run(PublisherListener<T> listener) {
                listener.onMasterRegistrationFailure(this);
            }
        });
    }

    public void signalOnMasterUnregistrationSuccess() {
        this.listeners.signal(new SignalRunnable<PublisherListener<T>>() {
            public void run(PublisherListener<T> listener) {
                listener.onMasterUnregistrationSuccess(this);
            }
        });
    }

    public void signalOnMasterUnregistrationFailure() {
        this.listeners.signal(new SignalRunnable<PublisherListener<T>>() {
            public void run(PublisherListener<T> listener) {
                listener.onMasterUnregistrationFailure(this);
            }
        });
    }

    private void signalOnNewSubscriber(final SubscriberIdentifier subscriberIdentifier) {
        this.listeners.signal(new SignalRunnable<PublisherListener<T>>() {
            public void run(PublisherListener<T> listener) {
                listener.onNewSubscriber(this, subscriberIdentifier);
            }
        });
    }

    private void signalOnShutdown(long timeout, TimeUnit unit) {
        try {
            this.listeners.signal(new SignalRunnable<PublisherListener<T>>() {
                public void run(PublisherListener<T> listener) {
                    listener.onShutdown(this);
                }
            }, timeout, unit);
        } catch (InterruptedException e) {
        }
    }

    public String toString() {
        return "Publisher<" + toDeclaration() + ">";
    }
}
