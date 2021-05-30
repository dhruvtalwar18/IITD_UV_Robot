package std_msgs;

import org.ros.internal.message.Message;

public interface UInt16 extends Message {
    public static final String _DEFINITION = "uint16 data\n";
    public static final String _TYPE = "std_msgs/UInt16";

    short getData();

    void setData(short s);
}
