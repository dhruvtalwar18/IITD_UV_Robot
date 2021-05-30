package std_msgs;

import org.ros.internal.message.Message;

public interface UInt8 extends Message {
    public static final String _DEFINITION = "uint8 data\n";
    public static final String _TYPE = "std_msgs/UInt8";

    byte getData();

    void setData(byte b);
}
