package org.ros.internal.message.field;

import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.internal.message.DefaultMessageDeserializer;
import org.ros.internal.message.DefaultMessageSerializer;
import org.ros.internal.message.Message;
import org.ros.message.MessageDeserializer;
import org.ros.message.MessageFactory;
import org.ros.message.MessageIdentifier;
import org.ros.message.MessageSerializer;

public class MessageFieldType implements FieldType {
    private final MessageDeserializer<Message> deserializer;
    private final MessageFactory messageFactory;
    private final MessageIdentifier messageIdentifier;
    private final MessageSerializer<Message> serializer = new DefaultMessageSerializer();

    public MessageFieldType(MessageIdentifier messageIdentifier2, MessageFactory messageFactory2) {
        this.messageIdentifier = messageIdentifier2;
        this.messageFactory = messageFactory2;
        this.deserializer = new DefaultMessageDeserializer(messageIdentifier2, messageFactory2);
    }

    public MessageFactory getMessageFactory() {
        return this.messageFactory;
    }

    public Field newVariableValue(String name) {
        return ValueField.newVariable(this, name);
    }

    public <T> Field newConstantValue(String name, T t) {
        throw new UnsupportedOperationException();
    }

    public Field newVariableList(String name, int size) {
        return ListField.newVariable(this, name);
    }

    public <T> T getDefaultValue() {
        return getMessageFactory().newFromType(this.messageIdentifier.getType());
    }

    public String getMd5String() {
        return null;
    }

    public String getJavaTypeName() {
        return String.format("%s.%s", new Object[]{this.messageIdentifier.getPackage(), this.messageIdentifier.getName()});
    }

    public int getSerializedSize() {
        throw new UnsupportedOperationException();
    }

    public String getName() {
        return this.messageIdentifier.getType();
    }

    public <T> void serialize(T value, ChannelBuffer buffer) {
        this.serializer.serialize((Message) value, buffer);
    }

    public Message deserialize(ChannelBuffer buffer) {
        return this.deserializer.deserialize(buffer);
    }

    public Void parseFromString(String value) {
        throw new UnsupportedOperationException();
    }

    public String toString() {
        return "MessageField<" + this.messageIdentifier + ">";
    }

    public int hashCode() {
        return (1 * 31) + (this.messageIdentifier == null ? 0 : this.messageIdentifier.hashCode());
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        MessageFieldType other = (MessageFieldType) obj;
        if (this.messageIdentifier == null) {
            if (other.messageIdentifier != null) {
                return false;
            }
        } else if (!this.messageIdentifier.equals(other.messageIdentifier)) {
            return false;
        }
        return true;
    }
}
