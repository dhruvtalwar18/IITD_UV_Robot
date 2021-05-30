package geometry_msgs;

import java.util.List;
import org.ros.internal.message.Message;
import std_msgs.Header;

public interface PoseArray extends Message {
    public static final String _DEFINITION = "# An array of poses with a header for global reference.\n\nHeader header\n\nPose[] poses\n";
    public static final String _TYPE = "geometry_msgs/PoseArray";

    Header getHeader();

    List<Pose> getPoses();

    void setHeader(Header header);

    void setPoses(List<Pose> list);
}
