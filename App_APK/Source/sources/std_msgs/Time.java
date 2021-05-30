package std_msgs;

import org.ros.internal.message.Message;

public interface Time extends Message {
    public static final String _DEFINITION = "time data\n";
    public static final String _TYPE = "std_msgs/Time";

    org.ros.message.Time getData();

    void setData(org.ros.message.Time time);
}
