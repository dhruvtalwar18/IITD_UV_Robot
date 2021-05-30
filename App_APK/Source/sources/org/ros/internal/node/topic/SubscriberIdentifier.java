package org.ros.internal.node.topic;

import com.google.common.base.Preconditions;
import java.net.URI;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.internal.transport.ConnectionHeader;
import org.ros.namespace.GraphName;

public class SubscriberIdentifier {
    private final NodeIdentifier nodeIdentifier;
    private final TopicIdentifier topicIdentifier;

    public SubscriberIdentifier(NodeIdentifier nodeIdentifier2, TopicIdentifier topicIdentifier2) {
        Preconditions.checkNotNull(nodeIdentifier2);
        Preconditions.checkNotNull(topicIdentifier2);
        this.nodeIdentifier = nodeIdentifier2;
        this.topicIdentifier = topicIdentifier2;
    }

    public ConnectionHeader toConnectionHeader() {
        ConnectionHeader connectionHeader = new ConnectionHeader();
        connectionHeader.merge(this.nodeIdentifier.toConnectionHeader());
        connectionHeader.merge(this.topicIdentifier.toConnectionHeader());
        return connectionHeader;
    }

    public NodeIdentifier getNodeIdentifier() {
        return this.nodeIdentifier;
    }

    public URI getUri() {
        return this.nodeIdentifier.getUri();
    }

    public TopicIdentifier getTopicIdentifier() {
        return this.topicIdentifier;
    }

    public GraphName getTopicName() {
        return this.topicIdentifier.getName();
    }

    public String toString() {
        return "SubscriberIdentifier<" + this.nodeIdentifier + ", " + this.topicIdentifier + ">";
    }

    public int hashCode() {
        int i = 0;
        int result = ((1 * 31) + (this.nodeIdentifier == null ? 0 : this.nodeIdentifier.hashCode())) * 31;
        if (this.topicIdentifier != null) {
            i = this.topicIdentifier.hashCode();
        }
        return result + i;
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        SubscriberIdentifier other = (SubscriberIdentifier) obj;
        if (this.nodeIdentifier == null) {
            if (other.nodeIdentifier != null) {
                return false;
            }
        } else if (!this.nodeIdentifier.equals(other.nodeIdentifier)) {
            return false;
        }
        if (this.topicIdentifier == null) {
            if (other.topicIdentifier != null) {
                return false;
            }
        } else if (!this.topicIdentifier.equals(other.topicIdentifier)) {
            return false;
        }
        return true;
    }
}
