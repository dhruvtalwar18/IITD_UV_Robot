package org.ros.internal.message;

import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.internal.message.field.Field;
import org.ros.message.MessageDeserializer;
import org.ros.message.MessageFactory;
import org.ros.message.MessageIdentifier;

public class DefaultMessageDeserializer<T> implements MessageDeserializer<T> {
    private final MessageFactory messageFactory;
    private final MessageIdentifier messageIdentifier;

    public DefaultMessageDeserializer(MessageIdentifier messageIdentifier2, MessageFactory messageFactory2) {
        this.messageIdentifier = messageIdentifier2;
        this.messageFactory = messageFactory2;
    }

    public T deserialize(ChannelBuffer buffer) {
        Message message = (Message) this.messageFactory.newFromType(this.messageIdentifier.getType());
        for (Field field : message.toRawMessage().getFields()) {
            if (!field.isConstant()) {
                field.deserialize(buffer);
            }
        }
        return message;
    }
}
