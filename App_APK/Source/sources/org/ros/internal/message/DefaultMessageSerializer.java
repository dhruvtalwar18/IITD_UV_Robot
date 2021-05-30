package org.ros.internal.message;

import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.internal.message.field.Field;
import org.ros.message.MessageSerializer;

public class DefaultMessageSerializer implements MessageSerializer<Message> {
    public void serialize(Message message, ChannelBuffer buffer) {
        for (Field field : message.toRawMessage().getFields()) {
            if (!field.isConstant()) {
                field.serialize(buffer);
            }
        }
    }
}
