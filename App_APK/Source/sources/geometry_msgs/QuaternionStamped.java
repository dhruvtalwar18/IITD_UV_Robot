package geometry_msgs;

import org.ros.internal.message.Message;
import std_msgs.Header;

public interface QuaternionStamped extends Message {
    public static final String _DEFINITION = "# This represents an orientation with reference coordinate frame and timestamp.\n\nHeader header\nQuaternion quaternion\n";
    public static final String _TYPE = "geometry_msgs/QuaternionStamped";

    Header getHeader();

    Quaternion getQuaternion();

    void setHeader(Header header);

    void setQuaternion(Quaternion quaternion);
}
