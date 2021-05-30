package tf2_msgs;

import org.ros.internal.message.Message;

public interface FrameGraphResponse extends Message {
    public static final String _DEFINITION = "string frame_yaml";
    public static final String _TYPE = "tf2_msgs/FrameGraphResponse";

    String getFrameYaml();

    void setFrameYaml(String str);
}
