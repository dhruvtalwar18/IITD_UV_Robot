package std_msgs;

import org.ros.internal.message.Message;

public interface Float32 extends Message {
    public static final String _DEFINITION = "float32 data";
    public static final String _TYPE = "std_msgs/Float32";

    float getData();

    void setData(float f);
}
