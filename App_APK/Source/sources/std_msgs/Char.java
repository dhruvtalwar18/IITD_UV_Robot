package std_msgs;

import org.ros.internal.message.Message;

public interface Char extends Message {
    public static final String _DEFINITION = "char data";
    public static final String _TYPE = "std_msgs/Char";

    byte getData();

    void setData(byte b);
}
