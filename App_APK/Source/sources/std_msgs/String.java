package std_msgs;

import org.ros.internal.message.Message;

public interface String extends Message {
    public static final String _DEFINITION = "string data\n";
    public static final String _TYPE = "std_msgs/String";

    String getData();

    void setData(String str);
}
