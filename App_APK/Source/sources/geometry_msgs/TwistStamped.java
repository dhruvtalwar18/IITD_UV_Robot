package geometry_msgs;

import org.ros.internal.message.Message;
import std_msgs.Header;

public interface TwistStamped extends Message {
    public static final String _DEFINITION = "# A twist with reference coordinate frame and timestamp\nHeader header\nTwist twist\n";
    public static final String _TYPE = "geometry_msgs/TwistStamped";

    Header getHeader();

    Twist getTwist();

    void setHeader(Header header);

    void setTwist(Twist twist);
}
