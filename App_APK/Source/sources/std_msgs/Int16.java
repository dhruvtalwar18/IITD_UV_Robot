package std_msgs;

import org.ros.internal.message.Message;

public interface Int16 extends Message {
    public static final String _DEFINITION = "int16 data\n";
    public static final String _TYPE = "std_msgs/Int16";

    short getData();

    void setData(short s);
}
