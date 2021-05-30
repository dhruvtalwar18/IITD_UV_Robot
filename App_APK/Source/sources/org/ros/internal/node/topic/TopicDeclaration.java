package org.ros.internal.node.topic;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import java.util.List;
import java.util.Map;
import org.ros.internal.message.topic.TopicDescription;
import org.ros.internal.transport.ConnectionHeader;
import org.ros.internal.transport.ConnectionHeaderFields;
import org.ros.namespace.GraphName;

public class TopicDeclaration {
    private final TopicDescription topicDescription;
    private final TopicIdentifier topicIdentifier;

    public static TopicDeclaration newFromHeader(Map<String, String> header) {
        Preconditions.checkArgument(header.containsKey(ConnectionHeaderFields.TOPIC));
        GraphName name = GraphName.of(header.get(ConnectionHeaderFields.TOPIC));
        return new TopicDeclaration(new TopicIdentifier(name), new TopicDescription(header.get(ConnectionHeaderFields.TYPE), header.get(ConnectionHeaderFields.MESSAGE_DEFINITION), header.get(ConnectionHeaderFields.MD5_CHECKSUM)));
    }

    public static TopicDeclaration newFromTopicName(GraphName topicName, TopicDescription topicDescription2) {
        return new TopicDeclaration(new TopicIdentifier(topicName), topicDescription2);
    }

    public TopicDeclaration(TopicIdentifier topicIdentifier2, TopicDescription topicDescription2) {
        Preconditions.checkNotNull(topicIdentifier2);
        Preconditions.checkNotNull(topicDescription2);
        this.topicIdentifier = topicIdentifier2;
        this.topicDescription = topicDescription2;
    }

    public TopicIdentifier getIdentifier() {
        return this.topicIdentifier;
    }

    public GraphName getName() {
        return this.topicIdentifier.getName();
    }

    public String getMessageType() {
        return this.topicDescription.getType();
    }

    public ConnectionHeader toConnectionHeader() {
        ConnectionHeader connectionHeader = new ConnectionHeader();
        connectionHeader.merge(this.topicIdentifier.toConnectionHeader());
        connectionHeader.addField(ConnectionHeaderFields.TYPE, this.topicDescription.getType());
        connectionHeader.addField(ConnectionHeaderFields.MESSAGE_DEFINITION, this.topicDescription.getDefinition());
        connectionHeader.addField(ConnectionHeaderFields.MD5_CHECKSUM, this.topicDescription.getMd5Checksum());
        return connectionHeader;
    }

    public List<String> toList() {
        return Lists.newArrayList((E[]) new String[]{getName().toString(), getMessageType()});
    }

    public String toString() {
        return "Topic<" + this.topicIdentifier + ", " + this.topicDescription.toString() + ">";
    }

    public int hashCode() {
        int i = 0;
        int result = ((1 * 31) + (this.topicDescription == null ? 0 : this.topicDescription.hashCode())) * 31;
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
        TopicDeclaration other = (TopicDeclaration) obj;
        if (this.topicDescription == null) {
            if (other.topicDescription != null) {
                return false;
            }
        } else if (!this.topicDescription.equals(other.topicDescription)) {
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
