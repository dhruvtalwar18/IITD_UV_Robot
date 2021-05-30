package org.ros.internal.node.server.master;

import com.google.common.collect.ImmutableSet;
import com.google.common.collect.Sets;
import java.net.URI;
import java.util.Set;
import org.ros.namespace.GraphName;

public class NodeRegistrationInfo {
    private final GraphName nodeName;
    private final URI nodeSlaveUri;
    private final Set<TopicRegistrationInfo> publishers = Sets.newHashSet();
    private final Set<ServiceRegistrationInfo> services = Sets.newHashSet();
    private final Set<TopicRegistrationInfo> subscribers = Sets.newHashSet();

    public NodeRegistrationInfo(GraphName nodeName2, URI nodeSlaveUri2) {
        this.nodeName = nodeName2;
        this.nodeSlaveUri = nodeSlaveUri2;
    }

    public GraphName getNodeName() {
        return this.nodeName;
    }

    public URI getNodeSlaveUri() {
        return this.nodeSlaveUri;
    }

    public boolean hasRegistrations() {
        return !this.publishers.isEmpty() || !this.subscribers.isEmpty() || !this.services.isEmpty();
    }

    public Set<TopicRegistrationInfo> getPublishers() {
        return ImmutableSet.copyOf(this.publishers);
    }

    public void addPublisher(TopicRegistrationInfo publisherTopic) {
        this.publishers.add(publisherTopic);
    }

    public boolean removePublisher(TopicRegistrationInfo publisherTopic) {
        return this.publishers.remove(publisherTopic);
    }

    public Set<TopicRegistrationInfo> getSubscribers() {
        return ImmutableSet.copyOf(this.subscribers);
    }

    public void addSubscriber(TopicRegistrationInfo subscriberTopic) {
        this.subscribers.add(subscriberTopic);
    }

    public boolean removeSubscriber(TopicRegistrationInfo subscriberTopic) {
        return this.subscribers.remove(subscriberTopic);
    }

    public Set<ServiceRegistrationInfo> getServices() {
        return ImmutableSet.copyOf(this.services);
    }

    public void addService(ServiceRegistrationInfo service) {
        this.services.add(service);
    }

    public boolean removeService(ServiceRegistrationInfo service) {
        return this.services.remove(service);
    }

    public int hashCode() {
        return (1 * 31) + this.nodeName.hashCode();
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj != null && getClass() == obj.getClass() && this.nodeName.equals(((NodeRegistrationInfo) obj).nodeName)) {
            return true;
        }
        return false;
    }
}
