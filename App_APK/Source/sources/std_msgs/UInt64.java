package std_msgs;

import org.ros.internal.message.Message;

public interface UInt64 extends Message {
    public static final String _DEFINITION = "uint64 data";
    public static final String _TYPE = "std_msgs/UInt64";

    long getData();

    void setData(long j);
}
