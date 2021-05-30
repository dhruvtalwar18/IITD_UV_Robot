package std_msgs;

import org.ros.internal.message.Message;

public interface Float64 extends Message {
    public static final String _DEFINITION = "float64 data";
    public static final String _TYPE = "std_msgs/Float64";

    double getData();

    void setData(double d);
}
