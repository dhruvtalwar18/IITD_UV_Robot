package org.ros.internal.message.field;

import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import org.ros.exception.RosMessageRuntimeException;
import org.ros.internal.message.context.MessageContext;

public class MessageFields {
    private final Map<String, Field> fields = Maps.newHashMap();
    private final Map<String, Field> getters = Maps.newHashMap();
    private final List<Field> orderedFields = Lists.newArrayList();
    private final Map<String, Field> setters = Maps.newHashMap();

    public MessageFields(MessageContext messageContext) {
        for (String name : messageContext.getFieldNames()) {
            Field field = messageContext.getFieldFactory(name).create();
            this.fields.put(name, field);
            this.getters.put(messageContext.getFieldGetterName(name), field);
            this.setters.put(messageContext.getFieldSetterName(name), field);
            this.orderedFields.add(field);
        }
    }

    public Field getField(String name) {
        return this.fields.get(name);
    }

    public Field getSetterField(String name) {
        return this.setters.get(name);
    }

    public Field getGetterField(String name) {
        return this.getters.get(name);
    }

    public List<Field> getFields() {
        return Collections.unmodifiableList(this.orderedFields);
    }

    public Object getFieldValue(String name) {
        Field field = this.fields.get(name);
        if (field != null) {
            return field.getValue();
        }
        throw new RosMessageRuntimeException("Uknown field: " + name);
    }

    public void setFieldValue(String name, Object value) {
        Field field = this.fields.get(name);
        if (field != null) {
            field.setValue(value);
            return;
        }
        throw new RosMessageRuntimeException("Uknown field: " + name);
    }

    public int hashCode() {
        int i = 0;
        int result = ((1 * 31) + (this.fields == null ? 0 : this.fields.hashCode())) * 31;
        if (this.orderedFields != null) {
            i = this.orderedFields.hashCode();
        }
        return result + i;
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        MessageFields other = (MessageFields) obj;
        if (this.fields == null) {
            if (other.fields != null) {
                return false;
            }
        } else if (!this.fields.equals(other.fields)) {
            return false;
        }
        if (this.orderedFields == null) {
            if (other.orderedFields != null) {
                return false;
            }
        } else if (!this.orderedFields.equals(other.orderedFields)) {
            return false;
        }
        return true;
    }
}
