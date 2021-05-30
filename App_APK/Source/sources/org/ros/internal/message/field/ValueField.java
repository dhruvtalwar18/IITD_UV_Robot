package org.ros.internal.message.field;

import com.google.common.base.Preconditions;
import org.jboss.netty.buffer.ChannelBuffer;

class ValueField<T> extends Field {
    private T value;

    static <T> ValueField<T> newConstant(FieldType type, String name, T value2) {
        return new ValueField<>(type, name, value2, true);
    }

    static <T> ValueField<T> newVariable(FieldType type, String name) {
        return new ValueField<>(type, name, (Object) null, false);
    }

    private ValueField(FieldType type, String name, T value2, boolean isConstant) {
        super(type, name, isConstant);
        this.value = value2;
    }

    public T getValue() {
        if (this.value == null) {
            setValue(this.type.getDefaultValue());
        }
        return this.value;
    }

    public void setValue(Object value2) {
        Preconditions.checkNotNull(value2);
        Preconditions.checkState(!this.isConstant);
        this.value = value2;
    }

    public void serialize(ChannelBuffer buffer) {
        this.type.serialize(getValue(), buffer);
    }

    public void deserialize(ChannelBuffer buffer) {
        Preconditions.checkState(!this.isConstant);
        setValue(this.type.deserialize(buffer));
    }

    public String getMd5String() {
        return String.format("%s %s\n", new Object[]{this.type, this.name});
    }

    public String getJavaTypeName() {
        return this.type.getJavaTypeName();
    }

    public String toString() {
        return "ValueField<" + this.type + ", " + this.name + ">";
    }

    public int hashCode() {
        return (super.hashCode() * 31) + (getValue() == null ? 0 : getValue().hashCode());
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!super.equals(obj) || getClass() != obj.getClass()) {
            return false;
        }
        Field other = (Field) obj;
        if (getValue() == null) {
            if (other.getValue() != null) {
                return false;
            }
        } else if (!getValue().equals(other.getValue())) {
            return false;
        }
        return true;
    }
}
