package geometry_msgs;

import org.ros.internal.message.Message;
import std_msgs.Header;

public interface TransformStamped extends Message {
    public static final String _DEFINITION = "# This expresses a transform from coordinate frame header.frame_id\n# to the coordinate frame child_frame_id\n#\n# This message is mostly used by the \n# <a href=\"http://wiki.ros.org/tf\">tf</a> package. \n# See its documentation for more information.\n\nHeader header\nstring child_frame_id # the frame id of the child frame\nTransform transform\n";
    public static final String _TYPE = "geometry_msgs/TransformStamped";

    String getChildFrameId();

    Header getHeader();

    Transform getTransform();

    void setChildFrameId(String str);

    void setHeader(Header header);

    void setTransform(Transform transform);
}
