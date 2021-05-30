package org.ros.internal.message;

import java.util.List;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.internal.message.field.Field;
import org.ros.message.Duration;
import org.ros.message.MessageIdentifier;
import org.ros.message.Time;

public interface RawMessage extends Message {
    boolean getBool(String str);

    boolean[] getBoolArray(String str);

    byte getByte(String str);

    byte[] getByteArray(String str);

    ChannelBuffer getChannelBuffer(String str);

    short getChar(String str);

    short[] getCharArray(String str);

    String getDefinition();

    Duration getDuration(String str);

    List<Duration> getDurationList(String str);

    List<Field> getFields();

    float getFloat32(String str);

    float[] getFloat32Array(String str);

    double getFloat64(String str);

    double[] getFloat64Array(String str);

    MessageIdentifier getIdentifier();

    short getInt16(String str);

    short[] getInt16Array(String str);

    int getInt32(String str);

    int[] getInt32Array(String str);

    long getInt64(String str);

    long[] getInt64Array(String str);

    byte getInt8(String str);

    ChannelBuffer getInt8Array(String str);

    <T extends Message> T getMessage(String str);

    <T extends Message> List<T> getMessageList(String str);

    String getName();

    String getPackage();

    String getString(String str);

    List<String> getStringList(String str);

    Time getTime(String str);

    List<Time> getTimeList(String str);

    String getType();

    short getUInt16(String str);

    short[] getUInt16Array(String str);

    int getUInt32(String str);

    int[] getUInt32Array(String str);

    long getUInt64(String str);

    long[] getUInt64Array(String str);

    short getUInt8(String str);

    short[] getUInt8Array(String str);

    void setBool(String str, boolean z);

    void setBoolArray(String str, boolean[] zArr);

    void setByte(String str, byte b);

    void setByteArray(String str, byte[] bArr);

    void setChannelBuffer(String str, ChannelBuffer channelBuffer);

    void setChar(String str, short s);

    void setCharArray(String str, short[] sArr);

    void setDuration(String str, Duration duration);

    void setDurationList(String str, List<Duration> list);

    void setFloat32(String str, float f);

    void setFloat32Array(String str, float[] fArr);

    void setFloat64(String str, double d);

    void setFloat64Array(String str, double[] dArr);

    void setInt16(String str, short s);

    void setInt16Array(String str, short[] sArr);

    void setInt32(String str, int i);

    void setInt32Array(String str, int[] iArr);

    void setInt64(String str, long j);

    void setInt64Array(String str, long[] jArr);

    void setInt8(String str, byte b);

    void setInt8Array(String str, byte[] bArr);

    void setMessage(String str, Message message);

    void setMessageList(String str, List<Message> list);

    void setString(String str, String str2);

    void setStringList(String str, List<String> list);

    void setTime(String str, Time time);

    void setTimeList(String str, List<Time> list);

    void setUInt16(String str, short s);

    void setUInt16Array(String str, short[] sArr);

    void setUInt32(String str, int i);

    void setUInt32Array(String str, int[] iArr);

    void setUInt64(String str, long j);

    void setUInt64Array(String str, long[] jArr);

    void setUInt8(String str, byte b);

    void setUInt8Array(String str, byte[] bArr);
}
