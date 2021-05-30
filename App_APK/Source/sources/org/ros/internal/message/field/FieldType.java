package org.ros.internal.message.field;

import org.jboss.netty.buffer.ChannelBuffer;

public interface FieldType {
    <T> T deserialize(ChannelBuffer channelBuffer);

    <T> T getDefaultValue();

    String getJavaTypeName();

    String getMd5String();

    String getName();

    int getSerializedSize();

    <T> Field newConstantValue(String str, T t);

    Field newVariableList(String str, int i);

    Field newVariableValue(String str);

    <T> T parseFromString(String str);

    <T> void serialize(T t, ChannelBuffer channelBuffer);
}
