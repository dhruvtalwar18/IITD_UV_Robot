package rosjava_test_msgs;

import org.ros.internal.message.Message;

public interface AddTwoIntsRequest extends Message {
    public static final String _DEFINITION = "int64 a\nint64 b\n";
    public static final String _TYPE = "rosjava_test_msgs/AddTwoIntsRequest";

    long getA();

    long getB();

    void setA(long j);

    void setB(long j);
}
