package visualization_msgs;

import geometry_msgs.Pose;
import org.ros.internal.message.Message;
import std_msgs.Header;

public interface InteractiveMarkerPose extends Message {
    public static final String _DEFINITION = "# Time/frame info.\nHeader header\n\n# Initial pose. Also, defines the pivot point for rotations.\ngeometry_msgs/Pose pose\n\n# Identifying string. Must be globally unique in\n# the topic that this message is sent through.\nstring name\n";
    public static final String _TYPE = "visualization_msgs/InteractiveMarkerPose";

    Header getHeader();

    String getName();

    Pose getPose();

    void setHeader(Header header);

    void setName(String str);

    void setPose(Pose pose);
}
