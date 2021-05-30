package std_msgs;

import org.ros.internal.message.Message;

public interface Int8 extends Message {
    public static final String _DEFINITION = "int8 data\n";
    public static final String _TYPE = "std_msgs/Int8";

    byte getData();

    void setData(byte b);
}
