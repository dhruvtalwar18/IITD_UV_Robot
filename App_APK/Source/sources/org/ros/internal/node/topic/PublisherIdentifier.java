package org.ros.internal.node.topic;

import com.google.common.base.Preconditions;
import com.google.common.collect.Sets;
import java.net.URI;
import java.util.Collection;
import java.util.Set;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.internal.transport.ConnectionHeader;
import org.ros.namespace.GraphName;

public class PublisherIdentifier {
    private final NodeIdentifier nodeIdentifier;
    private final TopicIdentifier topicIdentifier;

    public static Collection<PublisherIdentifier> newCollectionFromUris(Collection<URI> publisherUris, TopicDeclaration topicDeclaration) {
        Set<PublisherIdentifier> publishers = Sets.newHashSet();
        for (URI uri : publisherUris) {
            publishers.add(new PublisherIdentifier(new NodeIdentifier((GraphName) null, uri), topicDeclaration.getIdentifier()));
        }
        return publishers;
    }

    public static PublisherIdentifier newFromStrings(String nodeName, String uri, String topicName) {
        return new PublisherIdentifier(NodeIdentifier.forNameAndUri(nodeName, uri), TopicIdentifier.forName(topicName));
    }

    public PublisherIdentifier(NodeIdentifier nodeIdentifier2, TopicIdentifier topicIdentifier2) {
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

    public GraphName getNodeName() {
        return this.nodeIdentifier.getName();
    }

    public URI getNodeUri() {
        return this.nodeIdentifier.getUri();
    }

    public TopicIdentifier getTopicIdentifier() {
        return this.topicIdentifier;
    }

    public GraphName getTopicName() {
        return this.topicIdentifier.getName();
    }

    public String toString() {
        return "PublisherIdentifier<" + this.nodeIdentifier + ", " + this.topicIdentifier + ">";
    }

    public int hashCode() {
        return (((1 * 31) + this.nodeIdentifier.hashCode()) * 31) + this.topicIdentifier.hashCode();
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        PublisherIdentifier other = (PublisherIdentifier) obj;
        if (this.nodeIdentifier.equals(other.nodeIdentifier) && this.topicIdentifier.equals(other.topicIdentifier)) {
            return true;
        }
        return false;
    }
}
