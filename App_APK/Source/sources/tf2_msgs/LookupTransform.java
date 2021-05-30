package tf2_msgs;

import org.ros.internal.message.Message;

public interface LookupTransform extends Message {
    public static final String _DEFINITION = "#Simple API\nstring target_frame\nstring source_frame\ntime source_time\nduration timeout\n\n#Advanced API\ntime target_time\nstring fixed_frame\n\n#Whether or not to use the advanced API\nbool advanced\n\n---\ngeometry_msgs/TransformStamped transform\ntf2_msgs/TF2Error error\n---\n";
    public static final String _TYPE = "tf2_msgs/LookupTransform";
}
