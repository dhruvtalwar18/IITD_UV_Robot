package org.ros.internal.message.field;

import com.google.common.base.Preconditions;
import java.util.ArrayList;
import java.util.List;
import org.jboss.netty.buffer.ChannelBuffer;

public class ListField<T> extends Field {
    private List<T> value = new ArrayList();

    public static <T> ListField<T> newVariable(FieldType type, String name) {
        return new ListField<>(type, name);
    }

    private ListField(FieldType type, String name) {
        super(type, name, false);
    }

    public List<T> getValue() {
        return this.value;
    }

    public void setValue(Object value2) {
        Preconditions.checkNotNull(value2);
        this.value = (List) value2;
    }

    public void serialize(ChannelBuffer buffer) {
        buffer.writeInt(this.value.size());
        for (T v : this.value) {
            this.type.serialize(v, buffer);
        }
    }

    public void deserialize(ChannelBuffer buffer) {
        this.value.clear();
        int size = buffer.readInt();
        for (int i = 0; i < size; i++) {
            this.value.add(this.type.deserialize(buffer));
        }
    }

    public String getMd5String() {
        return String.format("%s %s\n", new Object[]{this.type, this.name});
    }

    public String getJavaTypeName() {
        return String.format("java.util.List<%s>", new Object[]{this.type.getJavaTypeName()});
    }

    public String toString() {
        return "ListField<" + this.type + ", " + this.name + ">";
    }

    public int hashCode() {
        return (super.hashCode() * 31) + (this.value == null ? 0 : this.value.hashCode());
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!super.equals(obj) || getClass() != obj.getClass()) {
            return false;
        }
        ListField other = (ListField) obj;
        if (this.value == null) {
            if (other.value != null) {
                return false;
            }
        } else if (!this.value.equals(other.value)) {
            return false;
        }
        return true;
    }
}
