package org.ros.internal.node.topic;

import com.google.common.base.Preconditions;
import org.ros.internal.transport.ConnectionHeader;
import org.ros.internal.transport.ConnectionHeaderFields;
import org.ros.namespace.GraphName;

public class TopicIdentifier {
    private final GraphName name;

    public static TopicIdentifier forName(String name2) {
        return new TopicIdentifier(GraphName.of(name2));
    }

    public TopicIdentifier(GraphName name2) {
        Preconditions.checkNotNull(name2);
        Preconditions.checkArgument(name2.isGlobal());
        this.name = name2;
    }

    public ConnectionHeader toConnectionHeader() {
        ConnectionHeader connectionHeader = new ConnectionHeader();
        connectionHeader.addField(ConnectionHeaderFields.TOPIC, this.name.toString());
        return connectionHeader;
    }

    public GraphName getName() {
        return this.name;
    }

    public String toString() {
        return "TopicIdentifier<" + this.name + ">";
    }

    public int hashCode() {
        return (1 * 31) + (this.name == null ? 0 : this.name.hashCode());
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        TopicIdentifier other = (TopicIdentifier) obj;
        if (this.name == null) {
            if (other.name != null) {
                return false;
            }
        } else if (!this.name.equals(other.name)) {
            return false;
        }
        return true;
    }
}
