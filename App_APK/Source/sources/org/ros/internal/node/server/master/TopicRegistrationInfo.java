package org.ros.internal.node.server.master;

import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableSet;
import com.google.common.collect.Sets;
import java.util.Set;
import org.ros.namespace.GraphName;

public class TopicRegistrationInfo {
    private boolean isPublisherDefinedMessageType = false;
    private String messageType;
    private final Set<NodeRegistrationInfo> publishers = Sets.newHashSet();
    private final Set<NodeRegistrationInfo> subscribers = Sets.newHashSet();
    private final GraphName topicName;

    public TopicRegistrationInfo(GraphName topicName2) {
        this.topicName = topicName2;
    }

    public GraphName getTopicName() {
        return this.topicName;
    }

    public String getMessageType() {
        return this.messageType;
    }

    public boolean hasPublishers() {
        return !this.publishers.isEmpty();
    }

    public boolean hasSubscribers() {
        return !this.subscribers.isEmpty();
    }

    public boolean hasRegistrations() {
        return hasPublishers() || hasSubscribers();
    }

    public Set<NodeRegistrationInfo> getPublishers() {
        return ImmutableSet.copyOf(this.publishers);
    }

    public void addPublisher(NodeRegistrationInfo publisher, String messageType2) {
        Preconditions.checkNotNull(publisher);
        this.publishers.add(publisher);
        setMessageType(messageType2, true);
    }

    public boolean removePublisher(NodeRegistrationInfo publisher) {
        return this.publishers.remove(publisher);
    }

    public Set<NodeRegistrationInfo> getSubscribers() {
        return ImmutableSet.copyOf(this.subscribers);
    }

    public void addSubscriber(NodeRegistrationInfo subscriber, String messageType2) {
        Preconditions.checkNotNull(subscriber);
        this.subscribers.add(subscriber);
        setMessageType(messageType2, false);
    }

    public boolean removeSubscriber(NodeRegistrationInfo subscriber) {
        return this.subscribers.remove(subscriber);
    }

    private void setMessageType(String topicMessageType, boolean isPublisher) {
        if (isPublisher) {
            this.messageType = topicMessageType;
            this.isPublisherDefinedMessageType = true;
        } else if ("*".equals(topicMessageType)) {
        } else {
            if (this.messageType == null) {
                this.messageType = topicMessageType;
            } else if (!this.isPublisherDefinedMessageType) {
                this.messageType = topicMessageType;
            }
        }
    }

    public int hashCode() {
        return (1 * 31) + this.topicName.hashCode();
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj != null && getClass() == obj.getClass() && this.topicName.equals(((TopicRegistrationInfo) obj).topicName)) {
            return true;
        }
        return false;
    }
}
