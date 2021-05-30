package geometry_msgs;

import org.ros.internal.message.Message;
import std_msgs.Header;

public interface Vector3Stamped extends Message {
    public static final String _DEFINITION = "# This represents a Vector3 with reference coordinate frame and timestamp\nHeader header\nVector3 vector\n";
    public static final String _TYPE = "geometry_msgs/Vector3Stamped";

    Header getHeader();

    Vector3 getVector();

    void setHeader(Header header);

    void setVector(Vector3 vector3);
}
