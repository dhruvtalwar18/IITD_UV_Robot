package org.ros.internal.message.field;

import com.google.common.base.Preconditions;
import java.util.Arrays;
import org.jboss.netty.buffer.ChannelBuffer;

public class LongArrayField extends Field {
    private final int size;
    private long[] value;

    public static LongArrayField newVariable(FieldType type, String name, int size2) {
        Preconditions.checkArgument(type.equals(PrimitiveFieldType.UINT32) || type.equals(PrimitiveFieldType.INT64) || type.equals(PrimitiveFieldType.UINT64));
        return new LongArrayField(type, name, size2);
    }

    private LongArrayField(FieldType type, String name, int size2) {
        super(type, name, false);
        this.size = size2;
        setValue(new long[Math.max(0, size2)]);
    }

    public long[] getValue() {
        return this.value;
    }

    public void setValue(Object value2) {
        Preconditions.checkArgument(this.size < 0 || ((long[]) value2).length == this.size);
        this.value = (long[]) value2;
    }

    public void serialize(ChannelBuffer buffer) {
        if (this.size < 0) {
            buffer.writeInt(this.value.length);
        }
        for (long v : this.value) {
            this.type.serialize(Long.valueOf(v), buffer);
        }
    }

    public void deserialize(ChannelBuffer buffer) {
        int currentSize = this.size;
        if (currentSize < 0) {
            currentSize = buffer.readInt();
        }
        this.value = new long[currentSize];
        for (int i = 0; i < currentSize; i++) {
            this.value[i] = buffer.readLong();
        }
    }

    public String getMd5String() {
        return String.format("%s %s\n", new Object[]{this.type, this.name});
    }

    public String getJavaTypeName() {
        return this.type.getJavaTypeName() + "[]";
    }

    public String toString() {
        return "LongArrayField<" + this.type + ", " + this.name + ">";
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
        LongArrayField other = (LongArrayField) obj;
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
