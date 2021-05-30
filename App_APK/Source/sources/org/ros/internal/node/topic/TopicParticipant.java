package org.ros.internal.node.topic;

import org.ros.namespace.GraphName;

public interface TopicParticipant {
    String getTopicMessageType();

    GraphName getTopicName();
}
