package std_msgs;

import org.ros.internal.message.Message;

public interface UInt32 extends Message {
    public static final String _DEFINITION = "uint32 data";
    public static final String _TYPE = "std_msgs/UInt32";

    int getData();

    void setData(int i);
}
