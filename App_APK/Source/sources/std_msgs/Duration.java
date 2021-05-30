package std_msgs;

import org.ros.internal.message.Message;

public interface Duration extends Message {
    public static final String _DEFINITION = "duration data\n";
    public static final String _TYPE = "std_msgs/Duration";

    org.ros.message.Duration getData();

    void setData(org.ros.message.Duration duration);
}
