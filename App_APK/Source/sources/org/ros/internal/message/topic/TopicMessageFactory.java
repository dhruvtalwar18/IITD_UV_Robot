package org.ros.internal.message.topic;

import org.ros.internal.message.DefaultMessageFactory;
import org.ros.message.MessageDefinitionProvider;

public class TopicMessageFactory extends DefaultMessageFactory {
    public TopicMessageFactory(MessageDefinitionProvider messageDefinitionProvider) {
        super(messageDefinitionProvider);
    }
}
