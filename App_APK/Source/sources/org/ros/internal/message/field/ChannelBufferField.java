package org.ros.internal.message.field;

import com.google.common.base.Preconditions;
import java.nio.ByteOrder;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.ros.internal.message.MessageBuffers;

public class ChannelBufferField extends Field {
    private final int size;
    private ChannelBuffer value = MessageBuffers.dynamicBuffer();

    public static ChannelBufferField newVariable(FieldType type, String name, int size2) {
        return new ChannelBufferField(type, name, size2);
    }

    private ChannelBufferField(FieldType type, String name, int size2) {
        super(type, name, false);
        this.size = size2;
    }

    public ChannelBuffer getValue() {
        return this.value.duplicate();
    }

    public void setValue(Object value2) {
        ChannelBuffer channelBufferValue = null;
        if (value2 instanceof byte[]) {
            channelBufferValue = ChannelBuffers.wrappedBuffer(ByteOrder.LITTLE_ENDIAN, byte[].class.cast(value2));
        } else if (value2 instanceof ChannelBuffer) {
            channelBufferValue = ChannelBuffer.class.cast(value2);
        }
        boolean z = false;
        Preconditions.checkArgument(channelBufferValue.order() == ByteOrder.LITTLE_ENDIAN);
        if (this.size < 0 || channelBufferValue.readableBytes() == this.size) {
            z = true;
        }
        Preconditions.checkArgument(z);
        this.value = channelBufferValue;
    }

    public void serialize(ChannelBuffer buffer) {
        if (this.size < 0) {
            buffer.writeInt(this.value.readableBytes());
        }
        buffer.writeBytes(this.value, 0, this.value.readableBytes());
    }

    public void deserialize(ChannelBuffer buffer) {
        int currentSize = this.size;
        if (currentSize < 0) {
            currentSize = buffer.readInt();
        }
        this.value = buffer.readSlice(currentSize);
    }

    public String getMd5String() {
        return String.format("%s %s\n", new Object[]{this.type, this.name});
    }

    public String getJavaTypeName() {
        return "org.jboss.netty.buffer.ChannelBuffer";
    }

    public String toString() {
        return "ChannelBufferField<" + this.type + ", " + this.name + ">";
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
        ChannelBufferField other = (ChannelBufferField) obj;
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
