package org.ros.internal.message.context;

import com.google.common.base.Preconditions;
import org.ros.internal.message.definition.MessageDefinitionParser;
import org.ros.internal.message.field.Field;
import org.ros.internal.message.field.FieldFactory;
import org.ros.internal.message.field.FieldType;
import org.ros.internal.message.field.MessageFieldType;
import org.ros.internal.message.field.PrimitiveFieldType;
import org.ros.message.MessageIdentifier;

class MessageContextBuilder implements MessageDefinitionParser.MessageDefinitionVisitor {
    private final MessageContext messageContext;

    public MessageContextBuilder(MessageContext context) {
        this.messageContext = context;
    }

    private FieldType getFieldType(String type) {
        Preconditions.checkArgument(!type.equals(this.messageContext.getType()), "Message definitions may not be self-referential.");
        if (PrimitiveFieldType.existsFor(type)) {
            return PrimitiveFieldType.valueOf(type.toUpperCase());
        }
        return new MessageFieldType(MessageIdentifier.of(type), this.messageContext.getMessageFactory());
    }

    public void variableValue(String type, final String name) {
        final FieldType fieldType = getFieldType(type);
        this.messageContext.addFieldFactory(name, new FieldFactory() {
            public Field create() {
                return fieldType.newVariableValue(name);
            }
        });
    }

    public void variableList(String type, final int size, final String name) {
        final FieldType fieldType = getFieldType(type);
        this.messageContext.addFieldFactory(name, new FieldFactory() {
            public Field create() {
                return fieldType.newVariableList(name, size);
            }
        });
    }

    public void constantValue(String type, final String name, final String value) {
        final FieldType fieldType = getFieldType(type);
        this.messageContext.addFieldFactory(name, new FieldFactory() {
            public Field create() {
                return fieldType.newConstantValue(name, fieldType.parseFromString(value));
            }
        });
    }
}
