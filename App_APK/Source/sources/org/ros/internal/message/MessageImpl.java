package org.ros.internal.message;

import java.util.List;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.exception.RosMessageRuntimeException;
import org.ros.internal.message.context.MessageContext;
import org.ros.internal.message.field.Field;
import org.ros.internal.message.field.MessageFieldType;
import org.ros.internal.message.field.MessageFields;
import org.ros.message.Duration;
import org.ros.message.MessageIdentifier;
import org.ros.message.Time;

class MessageImpl implements RawMessage, GetInstance {
    private final MessageContext messageContext;
    private final MessageFields messageFields;

    public MessageImpl(MessageContext messageContext2) {
        this.messageContext = messageContext2;
        this.messageFields = new MessageFields(messageContext2);
    }

    public MessageContext getMessageContext() {
        return this.messageContext;
    }

    public MessageFields getMessageFields() {
        return this.messageFields;
    }

    public RawMessage toRawMessage() {
        return this;
    }

    public MessageIdentifier getIdentifier() {
        return this.messageContext.getMessageIdentifer();
    }

    public String getType() {
        return this.messageContext.getType();
    }

    public String getPackage() {
        return this.messageContext.getPackage();
    }

    public String getName() {
        return this.messageContext.getName();
    }

    public String getDefinition() {
        return this.messageContext.getDefinition();
    }

    public List<Field> getFields() {
        return this.messageFields.getFields();
    }

    public boolean getBool(String name) {
        return ((Boolean) this.messageFields.getFieldValue(name)).booleanValue();
    }

    public boolean[] getBoolArray(String name) {
        return (boolean[]) this.messageFields.getFieldValue(name);
    }

    public Duration getDuration(String name) {
        return (Duration) this.messageFields.getFieldValue(name);
    }

    public List<Duration> getDurationList(String name) {
        return (List) this.messageFields.getFieldValue(name);
    }

    public float getFloat32(String name) {
        return ((Float) this.messageFields.getFieldValue(name)).floatValue();
    }

    public float[] getFloat32Array(String name) {
        return (float[]) this.messageFields.getFieldValue(name);
    }

    public double getFloat64(String name) {
        return ((Double) this.messageFields.getFieldValue(name)).doubleValue();
    }

    public double[] getFloat64Array(String name) {
        return (double[]) this.messageFields.getFieldValue(name);
    }

    public short getInt16(String name) {
        return ((Short) this.messageFields.getFieldValue(name)).shortValue();
    }

    public short[] getInt16Array(String name) {
        return (short[]) this.messageFields.getFieldValue(name);
    }

    public int getInt32(String name) {
        return ((Integer) this.messageFields.getFieldValue(name)).intValue();
    }

    public int[] getInt32Array(String name) {
        return (int[]) this.messageFields.getFieldValue(name);
    }

    public long getInt64(String name) {
        return ((Long) this.messageFields.getFieldValue(name)).longValue();
    }

    public long[] getInt64Array(String name) {
        return (long[]) this.messageFields.getFieldValue(name);
    }

    public byte getInt8(String name) {
        return ((Byte) this.messageFields.getFieldValue(name)).byteValue();
    }

    public ChannelBuffer getInt8Array(String name) {
        return (ChannelBuffer) this.messageFields.getFieldValue(name);
    }

    public <T extends Message> T getMessage(String name) {
        if (this.messageFields.getField(name).getType() instanceof MessageFieldType) {
            return (Message) this.messageFields.getField(name).getValue();
        }
        throw new RosMessageRuntimeException("Failed to access message field: " + name);
    }

    public <T extends Message> List<T> getMessageList(String name) {
        if (this.messageFields.getField(name).getType() instanceof MessageFieldType) {
            return (List) this.messageFields.getField(name).getValue();
        }
        throw new RosMessageRuntimeException("Failed to access list field: " + name);
    }

    public String getString(String name) {
        return (String) this.messageFields.getFieldValue(name);
    }

    public List<String> getStringList(String name) {
        return (List) this.messageFields.getFieldValue(name);
    }

    public Time getTime(String name) {
        return (Time) this.messageFields.getFieldValue(name);
    }

    public List<Time> getTimeList(String name) {
        return (List) this.messageFields.getFieldValue(name);
    }

    public short getUInt16(String name) {
        return ((Short) this.messageFields.getFieldValue(name)).shortValue();
    }

    public short[] getUInt16Array(String name) {
        return (short[]) this.messageFields.getFieldValue(name);
    }

    public int getUInt32(String name) {
        return ((Integer) this.messageFields.getFieldValue(name)).intValue();
    }

    public int[] getUInt32Array(String name) {
        return (int[]) this.messageFields.getFieldValue(name);
    }

    public long getUInt64(String name) {
        return ((Long) this.messageFields.getFieldValue(name)).longValue();
    }

    public long[] getUInt64Array(String name) {
        return (long[]) this.messageFields.getFieldValue(name);
    }

    public short getUInt8(String name) {
        return ((Short) this.messageFields.getFieldValue(name)).shortValue();
    }

    public short[] getUInt8Array(String name) {
        return (short[]) this.messageFields.getFieldValue(name);
    }

    public void setBool(String name, boolean value) {
        this.messageFields.setFieldValue(name, Boolean.valueOf(value));
    }

    public void setBoolArray(String name, boolean[] value) {
        this.messageFields.setFieldValue(name, value);
    }

    public void setDurationList(String name, List<Duration> value) {
        this.messageFields.setFieldValue(name, value);
    }

    public void setDuration(String name, Duration value) {
        this.messageFields.setFieldValue(name, value);
    }

    public void setFloat32(String name, float value) {
        this.messageFields.setFieldValue(name, Float.valueOf(value));
    }

    public void setFloat32Array(String name, float[] value) {
        this.messageFields.setFieldValue(name, value);
    }

    public void setFloat64(String name, double value) {
        this.messageFields.setFieldValue(name, Double.valueOf(value));
    }

    public void setFloat64Array(String name, double[] value) {
        this.messageFields.setFieldValue(name, value);
    }

    public void setInt16(String name, short value) {
        this.messageFields.setFieldValue(name, Short.valueOf(value));
    }

    public void setInt16Array(String name, short[] value) {
        this.messageFields.setFieldValue(name, value);
    }

    public void setInt32(String name, int value) {
        this.messageFields.setFieldValue(name, Integer.valueOf(value));
    }

    public void setInt32Array(String name, int[] value) {
        this.messageFields.setFieldValue(name, value);
    }

    public void setInt64(String name, long value) {
        this.messageFields.setFieldValue(name, Long.valueOf(value));
    }

    public void setInt64Array(String name, long[] value) {
        this.messageFields.setFieldValue(name, value);
    }

    public void setInt8(String name, byte value) {
        this.messageFields.setFieldValue(name, Byte.valueOf(value));
    }

    public void setInt8Array(String name, byte[] value) {
        this.messageFields.setFieldValue(name, value);
    }

    public void setMessage(String name, Message value) {
        this.messageFields.setFieldValue(name, value);
    }

    public void setMessageList(String name, List<Message> value) {
        this.messageFields.setFieldValue(name, value);
    }

    public void setString(String name, String value) {
        this.messageFields.setFieldValue(name, value);
    }

    public void setStringList(String name, List<String> value) {
        this.messageFields.setFieldValue(name, value);
    }

    public void setTime(String name, Time value) {
        this.messageFields.setFieldValue(name, value);
    }

    public void setTimeList(String name, List<Time> value) {
        this.messageFields.setFieldValue(name, value);
    }

    public void setUInt16(String name, short value) {
        this.messageFields.setFieldValue(name, Short.valueOf(value));
    }

    public void setUInt16Array(String name, short[] value) {
        this.messageFields.setFieldValue(name, value);
    }

    public void setUInt32(String name, int value) {
        this.messageFields.setFieldValue(name, Integer.valueOf(value));
    }

    public void setUInt32Array(String name, int[] value) {
        this.messageFields.setFieldValue(name, value);
    }

    public void setUInt64(String name, long value) {
        this.messageFields.setFieldValue(name, Long.valueOf(value));
    }

    public void setUInt64Array(String name, long[] value) {
        this.messageFields.setFieldValue(name, value);
    }

    public void setUInt8(String name, byte value) {
        this.messageFields.setFieldValue(name, Byte.valueOf(value));
    }

    public void setUInt8Array(String name, byte[] value) {
        this.messageFields.setFieldValue(name, value);
    }

    public byte getByte(String name) {
        return ((Byte) this.messageFields.getFieldValue(name)).byteValue();
    }

    public short getChar(String name) {
        return ((Short) this.messageFields.getFieldValue(name)).shortValue();
    }

    public void setByte(String name, byte value) {
        this.messageFields.setFieldValue(name, Byte.valueOf(value));
    }

    public void setChar(String name, short value) {
        this.messageFields.setFieldValue(name, Short.valueOf(value));
    }

    public void setByteArray(String name, byte[] value) {
        this.messageFields.setFieldValue(name, value);
    }

    public void setCharArray(String name, short[] value) {
        this.messageFields.setFieldValue(name, value);
    }

    public byte[] getByteArray(String name) {
        return (byte[]) this.messageFields.getFieldValue(name);
    }

    public short[] getCharArray(String name) {
        return (short[]) this.messageFields.getFieldValue(name);
    }

    public ChannelBuffer getChannelBuffer(String name) {
        return (ChannelBuffer) this.messageFields.getFieldValue(name);
    }

    public void setChannelBuffer(String name, ChannelBuffer value) {
        this.messageFields.setFieldValue(name, value);
    }

    public Object getInstance() {
        return this;
    }

    public String toString() {
        return String.format("MessageImpl<%s>", new Object[]{getType()});
    }

    public int hashCode() {
        int i = 0;
        int result = ((1 * 31) + (this.messageContext == null ? 0 : this.messageContext.hashCode())) * 31;
        if (this.messageFields != null) {
            i = this.messageFields.hashCode();
        }
        return result + i;
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || !(obj instanceof GetInstance)) {
            return false;
        }
        Object obj2 = ((GetInstance) obj).getInstance();
        if (getClass() != obj2.getClass()) {
            return false;
        }
        MessageImpl other = (MessageImpl) obj2;
        if (this.messageContext == null) {
            if (other.messageContext != null) {
                return false;
            }
        } else if (!this.messageContext.equals(other.messageContext)) {
            return false;
        }
        if (this.messageFields == null) {
            if (other.messageFields != null) {
                return false;
            }
        } else if (!this.messageFields.equals(other.messageFields)) {
            return false;
        }
        return true;
    }
}
