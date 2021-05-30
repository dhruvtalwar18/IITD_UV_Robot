package org.ros.internal.message.field;

import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableSet;
import java.nio.charset.Charset;
import org.apache.xmlrpc.serializer.BooleanSerializer;
import org.apache.xmlrpc.serializer.DoubleSerializer;
import org.apache.xmlrpc.serializer.FloatSerializer;
import org.apache.xmlrpc.serializer.I4Serializer;
import org.bytedeco.javacpp.opencv_stitching;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.message.Duration;
import org.ros.message.Time;

public enum PrimitiveFieldType implements FieldType {
    BOOL {
        public Boolean getDefaultValue() {
            return Boolean.FALSE;
        }

        public BooleanArrayField newVariableList(String name, int size) {
            return BooleanArrayField.newVariable(name, size);
        }

        public int getSerializedSize() {
            return 1;
        }

        public <T> void serialize(T value, ChannelBuffer buffer) {
            Preconditions.checkArgument(value instanceof Boolean);
            buffer.writeByte(((Boolean) value).booleanValue() ? (byte) 1 : 0);
        }

        public Boolean deserialize(ChannelBuffer buffer) {
            boolean z = true;
            if (buffer.readByte() != 1) {
                z = false;
            }
            return Boolean.valueOf(z);
        }

        public Boolean parseFromString(String value) {
            return Boolean.valueOf(value.equals("1"));
        }

        public String getJavaTypeName() {
            return BooleanSerializer.BOOLEAN_TAG;
        }
    },
    INT8 {
        public Byte getDefaultValue() {
            return (byte) 0;
        }

        public Field newVariableList(String name, int size) {
            return ChannelBufferField.newVariable(this, name, size);
        }

        public int getSerializedSize() {
            return 1;
        }

        public <T> void serialize(T value, ChannelBuffer buffer) {
            Preconditions.checkArgument(value instanceof Byte);
            buffer.writeByte(((Byte) value).byteValue());
        }

        public Byte deserialize(ChannelBuffer buffer) {
            return Byte.valueOf(buffer.readByte());
        }

        public Byte parseFromString(String value) {
            return Byte.valueOf(Byte.parseByte(value));
        }

        public String getJavaTypeName() {
            return "byte";
        }
    },
    BYTE {
        public Byte getDefaultValue() {
            return (Byte) INT8.getDefaultValue();
        }

        public Field newVariableList(String name, int size) {
            return ChannelBufferField.newVariable(this, name, size);
        }

        public int getSerializedSize() {
            return INT8.getSerializedSize();
        }

        public <T> void serialize(T value, ChannelBuffer buffer) {
            INT8.serialize(value, buffer);
        }

        public Byte deserialize(ChannelBuffer buffer) {
            return (Byte) INT8.deserialize(buffer);
        }

        public Byte parseFromString(String value) {
            return (Byte) INT8.parseFromString(value);
        }

        public String getJavaTypeName() {
            return INT8.getJavaTypeName();
        }
    },
    UINT8 {
        public Byte getDefaultValue() {
            return (Byte) INT8.getDefaultValue();
        }

        public Field newVariableList(String name, int size) {
            return ChannelBufferField.newVariable(this, name, size);
        }

        public int getSerializedSize() {
            return INT8.getSerializedSize();
        }

        public <T> void serialize(T value, ChannelBuffer buffer) {
            INT8.serialize(value, buffer);
        }

        public Byte deserialize(ChannelBuffer buffer) {
            return (Byte) INT8.deserialize(buffer);
        }

        public Byte parseFromString(String value) {
            return Byte.valueOf((byte) Short.parseShort(value));
        }

        public String getJavaTypeName() {
            return INT8.getJavaTypeName();
        }
    },
    CHAR {
        public Byte getDefaultValue() {
            return (Byte) UINT8.getDefaultValue();
        }

        public Field newVariableList(String name, int size) {
            return ChannelBufferField.newVariable(this, name, size);
        }

        public int getSerializedSize() {
            return UINT8.getSerializedSize();
        }

        public <T> void serialize(T value, ChannelBuffer buffer) {
            UINT8.serialize(value, buffer);
        }

        public Byte deserialize(ChannelBuffer buffer) {
            return (Byte) UINT8.deserialize(buffer);
        }

        public Byte parseFromString(String value) {
            return (Byte) UINT8.parseFromString(value);
        }

        public String getJavaTypeName() {
            return UINT8.getJavaTypeName();
        }
    },
    INT16 {
        public Short getDefaultValue() {
            return 0;
        }

        public Field newVariableList(String name, int size) {
            return ShortArrayField.newVariable(this, name, size);
        }

        public int getSerializedSize() {
            return 2;
        }

        public <T> void serialize(T value, ChannelBuffer buffer) {
            Preconditions.checkArgument(value instanceof Short);
            buffer.writeShort(((Short) value).shortValue());
        }

        public Short deserialize(ChannelBuffer buffer) {
            return Short.valueOf(buffer.readShort());
        }

        public Short parseFromString(String value) {
            return Short.valueOf(Short.parseShort(value));
        }

        public String getJavaTypeName() {
            return "short";
        }
    },
    UINT16 {
        public Short getDefaultValue() {
            return (Short) INT16.getDefaultValue();
        }

        public Field newVariableList(String name, int size) {
            return ShortArrayField.newVariable(this, name, size);
        }

        public int getSerializedSize() {
            return INT16.getSerializedSize();
        }

        public <T> void serialize(T value, ChannelBuffer buffer) {
            INT16.serialize(value, buffer);
        }

        public Short deserialize(ChannelBuffer buffer) {
            return (Short) INT16.deserialize(buffer);
        }

        public Short parseFromString(String value) {
            return Short.valueOf((short) Integer.parseInt(value));
        }

        public String getJavaTypeName() {
            return INT16.getJavaTypeName();
        }
    },
    INT32 {
        public Integer getDefaultValue() {
            return 0;
        }

        public Field newVariableList(String name, int size) {
            return IntegerArrayField.newVariable(this, name, size);
        }

        public int getSerializedSize() {
            return 4;
        }

        public <T> void serialize(T value, ChannelBuffer buffer) {
            Preconditions.checkArgument(value instanceof Integer);
            buffer.writeInt(((Integer) value).intValue());
        }

        public Integer deserialize(ChannelBuffer buffer) {
            return Integer.valueOf(buffer.readInt());
        }

        public Integer parseFromString(String value) {
            return Integer.valueOf(Integer.parseInt(value));
        }

        public String getJavaTypeName() {
            return I4Serializer.INT_TAG;
        }
    },
    UINT32 {
        public Integer getDefaultValue() {
            return (Integer) INT32.getDefaultValue();
        }

        public Field newVariableList(String name, int size) {
            return IntegerArrayField.newVariable(this, name, size);
        }

        public int getSerializedSize() {
            return INT32.getSerializedSize();
        }

        public <T> void serialize(T value, ChannelBuffer buffer) {
            INT32.serialize(value, buffer);
        }

        public Integer deserialize(ChannelBuffer buffer) {
            return (Integer) INT32.deserialize(buffer);
        }

        public Integer parseFromString(String value) {
            return Integer.valueOf((int) Long.parseLong(value));
        }

        public String getJavaTypeName() {
            return INT32.getJavaTypeName();
        }
    },
    INT64 {
        public Long getDefaultValue() {
            return 0L;
        }

        public Field newVariableList(String name, int size) {
            return LongArrayField.newVariable(this, name, size);
        }

        public int getSerializedSize() {
            return 8;
        }

        public <T> void serialize(T value, ChannelBuffer buffer) {
            Preconditions.checkArgument(value instanceof Long);
            buffer.writeLong(((Long) value).longValue());
        }

        public Long deserialize(ChannelBuffer buffer) {
            return Long.valueOf(buffer.readLong());
        }

        public Long parseFromString(String value) {
            return Long.valueOf(Long.parseLong(value));
        }

        public String getJavaTypeName() {
            return "long";
        }
    },
    UINT64 {
        public Long getDefaultValue() {
            return (Long) INT64.getDefaultValue();
        }

        public Field newVariableList(String name, int size) {
            return INT64.newVariableList(name, size);
        }

        public int getSerializedSize() {
            return INT64.getSerializedSize();
        }

        public <T> void serialize(T value, ChannelBuffer buffer) {
            INT64.serialize(value, buffer);
        }

        public Long deserialize(ChannelBuffer buffer) {
            return (Long) INT64.deserialize(buffer);
        }

        public Long parseFromString(String value) {
            return (Long) INT64.parseFromString(value);
        }

        public String getJavaTypeName() {
            return INT64.getJavaTypeName();
        }
    },
    FLOAT32 {
        public Float getDefaultValue() {
            return Float.valueOf(0.0f);
        }

        public Field newVariableList(String name, int size) {
            return FloatArrayField.newVariable(name, size);
        }

        public int getSerializedSize() {
            return 4;
        }

        public <T> void serialize(T value, ChannelBuffer buffer) {
            Preconditions.checkArgument(value instanceof Float);
            buffer.writeFloat(((Float) value).floatValue());
        }

        public Float deserialize(ChannelBuffer buffer) {
            return Float.valueOf(buffer.readFloat());
        }

        public Float parseFromString(String value) {
            return Float.valueOf(Float.parseFloat(value));
        }

        public String getJavaTypeName() {
            return FloatSerializer.FLOAT_TAG;
        }
    },
    FLOAT64 {
        public Double getDefaultValue() {
            return Double.valueOf(opencv_stitching.Stitcher.ORIG_RESOL);
        }

        public int getSerializedSize() {
            return 8;
        }

        public Field newVariableList(String name, int size) {
            return DoubleArrayField.newVariable(name, size);
        }

        public <T> void serialize(T value, ChannelBuffer buffer) {
            Preconditions.checkArgument(value instanceof Double);
            buffer.writeDouble(((Double) value).doubleValue());
        }

        public Double deserialize(ChannelBuffer buffer) {
            return Double.valueOf(buffer.readDouble());
        }

        public Double parseFromString(String value) {
            return Double.valueOf(Double.parseDouble(value));
        }

        public String getJavaTypeName() {
            return DoubleSerializer.DOUBLE_TAG;
        }
    },
    STRING {
        public String getDefaultValue() {
            return "";
        }

        public Field newVariableList(String name, int size) {
            return ListField.newVariable(this, name);
        }

        public int getSerializedSize() {
            throw new UnsupportedOperationException();
        }

        public <T> void serialize(T value, ChannelBuffer buffer) {
            Preconditions.checkArgument(value instanceof String);
            byte[] bytes = ((String) value).getBytes(PrimitiveFieldType.DEFAULT_CHARSET);
            buffer.writeInt(bytes.length);
            buffer.writeBytes(bytes);
        }

        public String deserialize(ChannelBuffer buffer) {
            return PrimitiveFieldType.DEFAULT_CHARSET.decode(buffer.readSlice(buffer.readInt()).toByteBuffer()).toString();
        }

        public String parseFromString(String value) {
            return value;
        }

        public String getJavaTypeName() {
            return "java.lang.String";
        }
    },
    TIME {
        public Time getDefaultValue() {
            return new Time();
        }

        public Field newVariableList(String name, int size) {
            return ListField.newVariable(this, name);
        }

        public int getSerializedSize() {
            return 8;
        }

        public <T> void serialize(T value, ChannelBuffer buffer) {
            Preconditions.checkArgument(value instanceof Time);
            buffer.writeInt(((Time) value).secs);
            buffer.writeInt(((Time) value).nsecs);
        }

        public Time deserialize(ChannelBuffer buffer) {
            return new Time(buffer.readInt(), buffer.readInt());
        }

        public Void parseFromString(String value) {
            throw new UnsupportedOperationException();
        }

        public String getJavaTypeName() {
            return Time.class.getName();
        }
    },
    DURATION {
        public Duration getDefaultValue() {
            return new Duration();
        }

        public Field newVariableList(String name, int size) {
            return ListField.newVariable(this, name);
        }

        public int getSerializedSize() {
            return 8;
        }

        public <T> void serialize(T value, ChannelBuffer buffer) {
            Preconditions.checkArgument(value instanceof Duration);
            buffer.writeInt(((Duration) value).secs);
            buffer.writeInt(((Duration) value).nsecs);
        }

        public Duration deserialize(ChannelBuffer buffer) {
            return new Duration(buffer.readInt(), buffer.readInt());
        }

        public Void parseFromString(String value) {
            throw new UnsupportedOperationException();
        }

        public String getJavaTypeName() {
            return Duration.class.getName();
        }
    };
    
    /* access modifiers changed from: private */
    public static final Charset DEFAULT_CHARSET = null;
    private static final ImmutableSet<String> TYPE_NAMES = null;

    static {
        int i;
        DEFAULT_CHARSET = Charset.forName("UTF-8");
        ImmutableSet.Builder<String> builder = ImmutableSet.builder();
        for (PrimitiveFieldType type : values()) {
            builder.add((Object) type.getName());
        }
        TYPE_NAMES = builder.build();
    }

    public static boolean existsFor(String name) {
        return TYPE_NAMES.contains(name);
    }

    public Field newVariableValue(String name) {
        return ValueField.newVariable(this, name);
    }

    public <T> Field newConstantValue(String name, T value) {
        return ValueField.newConstant(this, name, value);
    }

    public String getName() {
        return toString().toLowerCase();
    }

    public String getMd5String() {
        return getName();
    }
}
