package org.ros.internal.message;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import java.util.List;
import org.apache.commons.codec.digest.DigestUtils;
import org.ros.internal.message.definition.MessageDefinitionParser;
import org.ros.internal.message.definition.MessageDefinitionTupleParser;
import org.ros.internal.message.field.PrimitiveFieldType;
import org.ros.message.MessageDefinitionProvider;

public class Md5Generator {
    private final MessageDefinitionProvider messageDefinitionProvider;

    public Md5Generator(MessageDefinitionProvider messageDefinitionProvider2) {
        this.messageDefinitionProvider = messageDefinitionProvider2;
    }

    public String generate(String messageType) {
        String messageDefinition = this.messageDefinitionProvider.get(messageType);
        Preconditions.checkNotNull(messageDefinition, "No definition for message type: " + messageType);
        List<String> parts = MessageDefinitionTupleParser.parse(messageDefinition, -1);
        StringBuilder text = new StringBuilder();
        for (String part : parts) {
            text.append(generateText(messageType, part));
        }
        return DigestUtils.md5Hex(text.toString());
    }

    private String generateText(String messageType, String messageDefinition) {
        final List<String> constants = Lists.newArrayList();
        final List<String> variables = Lists.newArrayList();
        new MessageDefinitionParser(new MessageDefinitionParser.MessageDefinitionVisitor() {
            public void variableValue(String type, String name) {
                if (!PrimitiveFieldType.existsFor(type)) {
                    type = Md5Generator.this.generate(type);
                }
                variables.add(String.format("%s %s\n", new Object[]{type, name}));
            }

            public void variableList(String type, int size, String name) {
                if (!PrimitiveFieldType.existsFor(type)) {
                    String md5Checksum = Md5Generator.this.generate(type);
                    variables.add(String.format("%s %s\n", new Object[]{md5Checksum, name}));
                } else if (size != -1) {
                    variables.add(String.format("%s[%d] %s\n", new Object[]{type, Integer.valueOf(size), name}));
                } else {
                    variables.add(String.format("%s[] %s\n", new Object[]{type, name}));
                }
            }

            public void constantValue(String type, String name, String value) {
                constants.add(String.format("%s %s=%s\n", new Object[]{type, name, value}));
            }
        }).parse(messageType, messageDefinition);
        String text = "";
        for (String constant : constants) {
            text = text + constant;
        }
        for (String variable : variables) {
            text = text + variable;
        }
        return text.trim();
    }
}
