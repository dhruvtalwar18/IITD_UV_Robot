package org.ros.internal.message;

import com.google.common.base.Preconditions;
import com.google.common.collect.Sets;
import java.util.Set;
import org.apache.commons.httpclient.HttpState;
import org.apache.commons.httpclient.cookie.CookieSpec;
import org.apache.commons.lang.StringEscapeUtils;
import org.ros.exception.RosMessageRuntimeException;
import org.ros.internal.message.context.MessageContext;
import org.ros.internal.message.context.MessageContextProvider;
import org.ros.internal.message.field.Field;
import org.ros.internal.message.field.FieldType;
import org.ros.internal.message.field.MessageFields;
import org.ros.internal.message.field.PrimitiveFieldType;
import org.ros.message.MessageDeclaration;
import org.ros.message.MessageFactory;

public class MessageInterfaceBuilder {
    private boolean addConstantsAndMethods;
    private String interfaceName;
    private MessageDeclaration messageDeclaration;
    private String nestedContent;
    private String packageName;

    private static String escapeJava(String str) {
        return StringEscapeUtils.escapeJava(str).replace("\\/", CookieSpec.PATH_DELIM).replace("'", "\\'");
    }

    public MessageDeclaration getMessageDeclaration() {
        return this.messageDeclaration;
    }

    public MessageInterfaceBuilder setMessageDeclaration(MessageDeclaration messageDeclaration2) {
        Preconditions.checkNotNull(messageDeclaration2);
        this.messageDeclaration = messageDeclaration2;
        return this;
    }

    public String getPackageName() {
        return this.packageName;
    }

    public MessageInterfaceBuilder setPackageName(String packageName2) {
        this.packageName = packageName2;
        return this;
    }

    public String getInterfaceName() {
        return this.interfaceName;
    }

    public MessageInterfaceBuilder setInterfaceName(String interfaceName2) {
        Preconditions.checkNotNull(interfaceName2);
        this.interfaceName = interfaceName2;
        return this;
    }

    public boolean getAddConstantsAndMethods() {
        return this.addConstantsAndMethods;
    }

    public void setAddConstantsAndMethods(boolean enabled) {
        this.addConstantsAndMethods = enabled;
    }

    public String getNestedContent() {
        return this.nestedContent;
    }

    public void setNestedContent(String nestedContent2) {
        this.nestedContent = nestedContent2;
    }

    public String build(MessageFactory messageFactory) {
        Preconditions.checkNotNull(this.messageDeclaration);
        Preconditions.checkNotNull(this.interfaceName);
        StringBuilder builder = new StringBuilder();
        if (this.packageName != null) {
            builder.append(String.format("package %s;\n\n", new Object[]{this.packageName}));
        }
        builder.append(String.format("public interface %s extends org.ros.internal.message.Message {\n", new Object[]{this.interfaceName}));
        builder.append(String.format("  static final java.lang.String _TYPE = \"%s\";\n", new Object[]{this.messageDeclaration.getType()}));
        builder.append(String.format("  static final java.lang.String _DEFINITION = \"%s\";\n", new Object[]{escapeJava(this.messageDeclaration.getDefinition())}));
        if (this.addConstantsAndMethods) {
            MessageContext messageContext = new MessageContextProvider(messageFactory).get(this.messageDeclaration);
            appendConstants(messageContext, builder);
            appendSettersAndGetters(messageContext, builder);
        }
        if (this.nestedContent != null) {
            builder.append("\n");
            builder.append(this.nestedContent);
        }
        builder.append("}\n");
        return builder.toString();
    }

    private String getJavaValue(PrimitiveFieldType primitiveFieldType, String value) {
        switch (primitiveFieldType) {
            case BOOL:
                return Boolean.valueOf(!value.equals("0") && !value.equals(HttpState.PREEMPTIVE_DEFAULT)).toString();
            case FLOAT32:
                return value + "f";
            case STRING:
                return "\"" + escapeJava(value) + "\"";
            case BYTE:
            case CHAR:
            case INT8:
            case UINT8:
            case INT16:
            case UINT16:
            case INT32:
            case UINT32:
            case INT64:
            case UINT64:
            case FLOAT64:
                return value;
            default:
                throw new RosMessageRuntimeException("Unsupported PrimitiveFieldType: " + primitiveFieldType);
        }
    }

    private void appendConstants(MessageContext messageContext, StringBuilder builder) {
        for (Field field : new MessageFields(messageContext).getFields()) {
            if (field.isConstant()) {
                Preconditions.checkState(field.getType() instanceof PrimitiveFieldType);
                FieldType fieldType = field.getType();
                builder.append(String.format("  static final %s %s = %s;\n", new Object[]{fieldType.getJavaTypeName(), field.getName(), getJavaValue((PrimitiveFieldType) fieldType, field.getValue().toString())}));
            }
        }
    }

    private void appendSettersAndGetters(MessageContext messageContext, StringBuilder builder) {
        MessageFields messageFields = new MessageFields(messageContext);
        Set<String> getters = Sets.newHashSet();
        for (Field field : messageFields.getFields()) {
            if (!field.isConstant()) {
                String type = field.getJavaTypeName();
                String getter = messageContext.getFieldGetterName(field.getName());
                String setter = messageContext.getFieldSetterName(field.getName());
                if (!getters.contains(getter)) {
                    getters.add(getter);
                    builder.append(String.format("  %s %s();\n", new Object[]{type, getter}));
                    builder.append(String.format("  void %s(%s value);\n", new Object[]{setter, type}));
                }
            }
        }
    }
}
