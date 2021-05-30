package std_msgs;

import org.ros.internal.message.Message;

public interface Int32 extends Message {
    public static final String _DEFINITION = "int32 data";
    public static final String _TYPE = "std_msgs/Int32";

    int getData();

    void setData(int i);
}
