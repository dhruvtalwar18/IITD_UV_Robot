package org.ros.internal.message.topic;

import com.google.common.annotations.VisibleForTesting;
import java.util.Collection;
import org.ros.internal.message.StringResourceProvider;
import org.ros.message.MessageDefinitionProvider;
import org.ros.message.MessageIdentifier;

public class TopicDefinitionResourceProvider implements MessageDefinitionProvider {
    private final StringResourceProvider stringResourceProvider = new StringResourceProvider();

    private String topicTypeToResourceName(String topicType) {
        MessageIdentifier messageIdentifier = MessageIdentifier.of(topicType);
        return String.format("/%s/msg/%s.msg", new Object[]{messageIdentifier.getPackage(), messageIdentifier.getName()});
    }

    public String get(String topicType) {
        return this.stringResourceProvider.get(topicTypeToResourceName(topicType));
    }

    public boolean has(String topicType) {
        return this.stringResourceProvider.has(topicTypeToResourceName(topicType));
    }

    @VisibleForTesting
    public void add(String topicType, String topicDefinition) {
        this.stringResourceProvider.addStringToCache(topicTypeToResourceName(topicType), topicDefinition);
    }

    public Collection<String> getPackages() {
        throw new UnsupportedOperationException();
    }

    public Collection<MessageIdentifier> getMessageIdentifiersByPackage(String pkg) {
        throw new UnsupportedOperationException();
    }
}
