package std_msgs;

import org.ros.internal.message.Message;

public interface Int64 extends Message {
    public static final String _DEFINITION = "int64 data";
    public static final String _TYPE = "std_msgs/Int64";

    long getData();

    void setData(long j);
}
