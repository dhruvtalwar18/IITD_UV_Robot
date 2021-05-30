package std_msgs;

import org.ros.internal.message.Message;

public interface Byte extends Message {
    public static final String _DEFINITION = "byte data\n";
    public static final String _TYPE = "std_msgs/Byte";

    byte getData();

    void setData(byte b);
}
