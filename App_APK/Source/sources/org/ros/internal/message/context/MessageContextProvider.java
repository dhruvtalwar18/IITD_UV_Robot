package org.ros.internal.message.context;

import com.google.common.base.Preconditions;
import com.google.common.collect.Maps;
import java.util.Map;
import org.ros.internal.message.definition.MessageDefinitionParser;
import org.ros.message.MessageDeclaration;
import org.ros.message.MessageFactory;

public class MessageContextProvider {
    private final Map<MessageDeclaration, MessageContext> cache = Maps.newConcurrentMap();
    private final MessageFactory messageFactory;

    public MessageContextProvider(MessageFactory messageFactory2) {
        Preconditions.checkNotNull(messageFactory2);
        this.messageFactory = messageFactory2;
    }

    public MessageContext get(MessageDeclaration messageDeclaration) {
        MessageContext messageContext = this.cache.get(messageDeclaration);
        if (messageContext != null) {
            return messageContext;
        }
        MessageContext messageContext2 = new MessageContext(messageDeclaration, this.messageFactory);
        new MessageDefinitionParser(new MessageContextBuilder(messageContext2)).parse(messageDeclaration.getType(), messageDeclaration.getDefinition());
        this.cache.put(messageDeclaration, messageContext2);
        return messageContext2;
    }
}
