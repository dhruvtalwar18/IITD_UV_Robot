package org.ros.internal.message.field;

import com.google.common.base.Preconditions;
import java.util.Arrays;
import org.jboss.netty.buffer.ChannelBuffer;

public class IntegerArrayField extends Field {
    private final int size;
    private int[] value;

    public static IntegerArrayField newVariable(FieldType type, String name, int size2) {
        return new IntegerArrayField(type, name, size2);
    }

    private IntegerArrayField(FieldType type, String name, int size2) {
        super(type, name, false);
        this.size = size2;
        setValue(new int[Math.max(0, size2)]);
    }

    public int[] getValue() {
        return this.value;
    }

    public void setValue(Object value2) {
        Preconditions.checkArgument(this.size < 0 || ((int[]) value2).length == this.size);
        this.value = (int[]) value2;
    }

    public void serialize(ChannelBuffer buffer) {
        if (this.size < 0) {
            buffer.writeInt(this.value.length);
        }
        for (int v : this.value) {
            this.type.serialize(Integer.valueOf(v), buffer);
        }
    }

    public void deserialize(ChannelBuffer buffer) {
        int currentSize = this.size;
        if (currentSize < 0) {
            currentSize = buffer.readInt();
        }
        this.value = new int[currentSize];
        for (int i = 0; i < currentSize; i++) {
            this.value[i] = buffer.readInt();
        }
    }

    public String getMd5String() {
        return String.format("%s %s\n", new Object[]{this.type, this.name});
    }

    public String getJavaTypeName() {
        return this.type.getJavaTypeName() + "[]";
    }

    public String toString() {
        return "IntegerArrayField<" + this.type + ", " + this.name + ">";
    }

    public int hashCode() {
        return (super.hashCode() * 31) + (this.value == null ? 0 : Arrays.hashCode(this.value));
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!super.equals(obj) || getClass() != obj.getClass()) {
            return false;
        }
        IntegerArrayField other = (IntegerArrayField) obj;
        if (this.value == null) {
            if (other.value != null) {
                return false;
            }
        } else if (!Arrays.equals(this.value, other.value)) {
            return false;
        }
        return true;
    }
}
