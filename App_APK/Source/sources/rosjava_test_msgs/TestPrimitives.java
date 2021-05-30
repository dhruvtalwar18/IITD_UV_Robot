package rosjava_test_msgs;

import org.ros.internal.message.Message;
import org.ros.message.Duration;
import org.ros.message.Time;

public interface TestPrimitives extends Message {
    public static final String _DEFINITION = "# Integration test message of all primitive types\n\n# caller_id of most recent node to send this message\nstring caller_id\n# caller_id of the original node to send this message\nstring orig_caller_id\n\nstring str\nbyte b\nint16 int16\nint32 int32\nint64 int64\nchar c\nuint16 uint16\nuint32 uint32\nuint64 uint64\nfloat32 float32\nfloat64 float64\ntime t\nduration d\n\n";
    public static final String _TYPE = "rosjava_test_msgs/TestPrimitives";

    byte getB();

    byte getC();

    String getCallerId();

    Duration getD();

    float getFloat32();

    double getFloat64();

    short getInt16();

    int getInt32();

    long getInt64();

    String getOrigCallerId();

    String getStr();

    Time getT();

    short getUint16();

    int getUint32();

    long getUint64();

    void setB(byte b);

    void setC(byte b);

    void setCallerId(String str);

    void setD(Duration duration);

    void setFloat32(float f);

    void setFloat64(double d);

    void setInt16(short s);

    void setInt32(int i);

    void setInt64(long j);

    void setOrigCallerId(String str);

    void setStr(String str);

    void setT(Time time);

    void setUint16(short s);

    void setUint32(int i);

    void setUint64(long j);
}
