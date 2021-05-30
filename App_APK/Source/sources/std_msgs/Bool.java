package std_msgs;

import org.ros.internal.message.Message;

public interface Bool extends Message {
    public static final String _DEFINITION = "bool data";
    public static final String _TYPE = "std_msgs/Bool";

    boolean getData();

    void setData(boolean z);
}
