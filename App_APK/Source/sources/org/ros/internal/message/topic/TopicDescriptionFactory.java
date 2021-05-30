package org.ros.internal.message.topic;

import org.ros.internal.message.Md5Generator;
import org.ros.message.MessageDefinitionProvider;

public class TopicDescriptionFactory {
    private final Md5Generator md5Generator;
    private final MessageDefinitionProvider messageDefinitionProvider;

    public TopicDescriptionFactory(MessageDefinitionProvider messageDefinitionProvider2) {
        this.messageDefinitionProvider = messageDefinitionProvider2;
        this.md5Generator = new Md5Generator(messageDefinitionProvider2);
    }

    public TopicDescription newFromType(String topicType) {
        return new TopicDescription(topicType, this.messageDefinitionProvider.get(topicType), this.md5Generator.generate(topicType));
    }

    public boolean hasType(String topicType) {
        return this.messageDefinitionProvider.has(topicType);
    }
}
