package org.ros.internal.message.definition;

import com.google.common.collect.Lists;
import com.google.common.collect.Sets;
import java.util.Collection;
import java.util.NoSuchElementException;
import java.util.Set;
import org.ros.message.MessageDefinitionProvider;
import org.ros.message.MessageIdentifier;

public class MessageDefinitionProviderChain implements MessageDefinitionProvider {
    private final Collection<MessageDefinitionProvider> messageDefinitionProviders = Lists.newArrayList();

    public void addMessageDefinitionProvider(MessageDefinitionProvider messageDefinitionProvider) {
        this.messageDefinitionProviders.add(messageDefinitionProvider);
    }

    public String get(String messageType) {
        for (MessageDefinitionProvider messageDefinitionProvider : this.messageDefinitionProviders) {
            if (messageDefinitionProvider.has(messageType)) {
                return messageDefinitionProvider.get(messageType);
            }
        }
        throw new NoSuchElementException("No message definition available for: " + messageType);
    }

    public boolean has(String messageType) {
        for (MessageDefinitionProvider messageDefinitionProvider : this.messageDefinitionProviders) {
            if (messageDefinitionProvider.has(messageType)) {
                return true;
            }
        }
        return false;
    }

    public Collection<String> getPackages() {
        Set<String> result = Sets.newHashSet();
        for (MessageDefinitionProvider messageDefinitionProvider : this.messageDefinitionProviders) {
            result.addAll(messageDefinitionProvider.getPackages());
        }
        return result;
    }

    public Collection<MessageIdentifier> getMessageIdentifiersByPackage(String pkg) {
        Set<MessageIdentifier> result = Sets.newHashSet();
        for (MessageDefinitionProvider messageDefinitionProvider : this.messageDefinitionProviders) {
            Collection<MessageIdentifier> messageIdentifiers = messageDefinitionProvider.getMessageIdentifiersByPackage(pkg);
            if (messageIdentifiers != null) {
                result.addAll(messageIdentifiers);
            }
        }
        return result;
    }
}
