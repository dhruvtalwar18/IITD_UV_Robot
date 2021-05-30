package tf2_msgs;

import geometry_msgs.TransformStamped;
import java.util.List;
import org.ros.internal.message.Message;

public interface TFMessage extends Message {
    public static final String _DEFINITION = "geometry_msgs/TransformStamped[] transforms\n";
    public static final String _TYPE = "tf2_msgs/TFMessage";

    List<TransformStamped> getTransforms();

    void setTransforms(List<TransformStamped> list);
}
