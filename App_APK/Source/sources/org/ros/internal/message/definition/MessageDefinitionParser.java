package org.ros.internal.message.definition;

import com.google.common.base.Preconditions;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.StringReader;
import org.apache.commons.httpclient.cookie.CookieSpec;
import org.ros.exception.RosMessageRuntimeException;
import org.ros.internal.message.field.PrimitiveFieldType;
import std_msgs.Header;

public class MessageDefinitionParser {
    private final MessageDefinitionVisitor visitor;

    public interface MessageDefinitionVisitor {
        void constantValue(String str, String str2, String str3);

        void variableList(String str, int i, String str2);

        void variableValue(String str, String str2);
    }

    public MessageDefinitionParser(MessageDefinitionVisitor visitor2) {
        this.visitor = visitor2;
    }

    public void parse(String messageType, String messageDefinition) {
        Preconditions.checkNotNull(messageType);
        Preconditions.checkNotNull(messageDefinition);
        BufferedReader reader = new BufferedReader(new StringReader(messageDefinition));
        while (true) {
            try {
                String line = reader.readLine();
                if (line != null) {
                    String line2 = line.trim();
                    if (!line2.startsWith("#")) {
                        if (line2.length() > 0) {
                            parseField(messageType, line2);
                        }
                    }
                } else {
                    return;
                }
            } catch (IOException e) {
                throw new RosMessageRuntimeException((Throwable) e);
            }
        }
    }

    private void parseField(String messageType, String fieldDefinition) {
        String[] typeAndName = fieldDefinition.split("\\s+", 2);
        Preconditions.checkState(typeAndName.length == 2, String.format("Invalid field definition: \"%s\"", new Object[]{fieldDefinition}));
        String type = typeAndName[0];
        String name = typeAndName[1];
        String value = null;
        if (name.contains("=") && (!name.contains("#") || name.indexOf(35) > name.indexOf(61))) {
            String[] nameAndValue = name.split("=", 2);
            name = nameAndValue[0].trim();
            value = nameAndValue[1].trim();
        } else if (name.contains("#")) {
            Preconditions.checkState(!name.startsWith("#"), String.format("Fields must define a name. Field definition in %s was: \"%s\"", new Object[]{messageType, fieldDefinition}));
            name = name.substring(0, name.indexOf(35)).trim();
        }
        boolean array = false;
        int size = -1;
        if (type.endsWith("]")) {
            int leftBracketIndex = type.lastIndexOf(91);
            int rightBracketIndex = type.lastIndexOf(93);
            array = true;
            if (rightBracketIndex - leftBracketIndex > 1) {
                size = Integer.parseInt(type.substring(leftBracketIndex + 1, rightBracketIndex));
            }
            type = type.substring(0, leftBracketIndex);
        }
        if (type.equals("Header")) {
            Preconditions.checkState(name.equals("header"), "Header field must be named \"header.\"");
            type = Header._TYPE;
        } else if (!PrimitiveFieldType.existsFor(type) && !type.contains(CookieSpec.PATH_DELIM)) {
            type = messageType.substring(0, messageType.lastIndexOf(47) + 1) + type;
        }
        if (value != null) {
            if (!array) {
                if (!type.equals(PrimitiveFieldType.STRING.getName()) && value.contains("#")) {
                    Preconditions.checkState(true ^ value.startsWith("#"), "Constants must define a value.");
                    value = value.substring(0, value.indexOf(35)).trim();
                }
                this.visitor.constantValue(type, name, value);
                return;
            }
            throw new UnsupportedOperationException("Array constants are not supported.");
        } else if (array) {
            this.visitor.variableList(type, size, name);
        } else {
            this.visitor.variableValue(type, name);
        }
    }
}
