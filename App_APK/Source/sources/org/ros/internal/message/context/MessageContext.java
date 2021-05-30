package org.ros.internal.message.context;

import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import org.ros.internal.message.field.FieldFactory;
import org.ros.message.MessageDeclaration;
import org.ros.message.MessageFactory;
import org.ros.message.MessageIdentifier;

public class MessageContext {
    private final Map<String, FieldFactory> fieldFactories = Maps.newHashMap();
    private final Map<String, String> fieldGetterNames = Maps.newHashMap();
    private final List<String> fieldNames = Lists.newArrayList();
    private final Map<String, String> fieldSetterNames = Maps.newHashMap();
    private final MessageDeclaration messageDeclaration;
    private final MessageFactory messageFactory;

    public MessageContext(MessageDeclaration messageDeclaration2, MessageFactory messageFactory2) {
        this.messageDeclaration = messageDeclaration2;
        this.messageFactory = messageFactory2;
    }

    public MessageFactory getMessageFactory() {
        return this.messageFactory;
    }

    public MessageIdentifier getMessageIdentifer() {
        return this.messageDeclaration.getMessageIdentifier();
    }

    public String getType() {
        return this.messageDeclaration.getType();
    }

    public String getPackage() {
        return this.messageDeclaration.getPackage();
    }

    public String getName() {
        return this.messageDeclaration.getName();
    }

    public String getDefinition() {
        return this.messageDeclaration.getDefinition();
    }

    public void addFieldFactory(String name, FieldFactory fieldFactory) {
        this.fieldFactories.put(name, fieldFactory);
        Map<String, String> map = this.fieldGetterNames;
        map.put(name, "get" + getJavaName(name));
        Map<String, String> map2 = this.fieldSetterNames;
        map2.put(name, "set" + getJavaName(name));
        this.fieldNames.add(name);
    }

    private String getJavaName(String name) {
        String[] parts = name.split("_");
        StringBuilder fieldName = new StringBuilder();
        for (String part : parts) {
            fieldName.append(part.substring(0, 1).toUpperCase() + part.substring(1));
        }
        return fieldName.toString();
    }

    public boolean hasField(String name) {
        return this.fieldFactories.containsKey(name);
    }

    public String getFieldGetterName(String name) {
        return this.fieldGetterNames.get(name);
    }

    public String getFieldSetterName(String name) {
        return this.fieldSetterNames.get(name);
    }

    public FieldFactory getFieldFactory(String name) {
        return this.fieldFactories.get(name);
    }

    public List<String> getFieldNames() {
        return Collections.unmodifiableList(this.fieldNames);
    }

    public int hashCode() {
        return (1 * 31) + (this.messageDeclaration == null ? 0 : this.messageDeclaration.hashCode());
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        MessageContext other = (MessageContext) obj;
        if (this.messageDeclaration == null) {
            if (other.messageDeclaration != null) {
                return false;
            }
        } else if (!this.messageDeclaration.equals(other.messageDeclaration)) {
            return false;
        }
        return true;
    }
}
