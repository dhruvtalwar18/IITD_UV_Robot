package org.ros.internal.node.topic;

import java.util.List;
import org.ros.internal.transport.ConnectionHeader;
import org.ros.namespace.GraphName;

public abstract class DefaultTopicParticipant implements TopicParticipant {
    private final TopicDeclaration topicDeclaration;

    public abstract void signalOnMasterRegistrationFailure();

    public abstract void signalOnMasterRegistrationSuccess();

    public abstract void signalOnMasterUnregistrationFailure();

    public abstract void signalOnMasterUnregistrationSuccess();

    public DefaultTopicParticipant(TopicDeclaration topicDeclaration2) {
        this.topicDeclaration = topicDeclaration2;
    }

    public TopicDeclaration getTopicDeclaration() {
        return this.topicDeclaration;
    }

    public List<String> getTopicDeclarationAsList() {
        return this.topicDeclaration.toList();
    }

    public GraphName getTopicName() {
        return this.topicDeclaration.getName();
    }

    public String getTopicMessageType() {
        return this.topicDeclaration.getMessageType();
    }

    public ConnectionHeader getTopicDeclarationHeader() {
        return this.topicDeclaration.toConnectionHeader();
    }
}
