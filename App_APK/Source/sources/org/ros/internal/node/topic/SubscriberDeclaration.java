package org.ros.internal.node.topic;

import java.net.URI;
import java.util.Map;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.internal.transport.ConnectionHeader;
import org.ros.internal.transport.ConnectionHeaderFields;
import org.ros.namespace.GraphName;

public class SubscriberDeclaration {
    private final SubscriberIdentifier subscriberIdentifier;
    private final TopicDeclaration topicDeclaration;

    public static SubscriberDeclaration newFromHeader(Map<String, String> header) {
        NodeIdentifier nodeIdentifier = new NodeIdentifier(GraphName.of(header.get(ConnectionHeaderFields.CALLER_ID)), (URI) null);
        TopicDeclaration topicDeclaration2 = TopicDeclaration.newFromHeader(header);
        return new SubscriberDeclaration(new SubscriberIdentifier(nodeIdentifier, topicDeclaration2.getIdentifier()), topicDeclaration2);
    }

    public SubscriberDeclaration(SubscriberIdentifier subscriberIdentifier2, TopicDeclaration topicDeclaration2) {
        this.subscriberIdentifier = subscriberIdentifier2;
        this.topicDeclaration = topicDeclaration2;
    }

    public NodeIdentifier getNodeIdentifier() {
        return this.subscriberIdentifier.getNodeIdentifier();
    }

    public URI getSlaveUri() {
        return this.subscriberIdentifier.getUri();
    }

    public GraphName getTopicName() {
        return this.topicDeclaration.getName();
    }

    public ConnectionHeader toConnectionHeader() {
        ConnectionHeader connectionHeader = new ConnectionHeader();
        connectionHeader.merge(this.subscriberIdentifier.toConnectionHeader());
        connectionHeader.merge(this.topicDeclaration.toConnectionHeader());
        return connectionHeader;
    }

    public String toString() {
        return "SubscriberDefinition<" + this.subscriberIdentifier + ", " + this.topicDeclaration + ">";
    }

    public int hashCode() {
        int i = 0;
        int result = ((1 * 31) + (this.subscriberIdentifier == null ? 0 : this.subscriberIdentifier.hashCode())) * 31;
        if (this.topicDeclaration != null) {
            i = this.topicDeclaration.hashCode();
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
        SubscriberDeclaration other = (SubscriberDeclaration) obj;
        if (this.subscriberIdentifier == null) {
            if (other.subscriberIdentifier != null) {
                return false;
            }
        } else if (!this.subscriberIdentifier.equals(other.subscriberIdentifier)) {
            return false;
        }
        if (this.topicDeclaration == null) {
            if (other.topicDeclaration != null) {
                return false;
            }
        } else if (!this.topicDeclaration.equals(other.topicDeclaration)) {
            return false;
        }
        return true;
    }
}
