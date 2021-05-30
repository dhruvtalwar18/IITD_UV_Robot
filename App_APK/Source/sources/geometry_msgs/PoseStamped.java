package geometry_msgs;

import org.ros.internal.message.Message;
import std_msgs.Header;

public interface PoseStamped extends Message {
    public static final String _DEFINITION = "# A Pose with reference coordinate frame and timestamp\nHeader header\nPose pose\n";
    public static final String _TYPE = "geometry_msgs/PoseStamped";

    Header getHeader();

    Pose getPose();

    void setHeader(Header header);

    void setPose(Pose pose);
}
