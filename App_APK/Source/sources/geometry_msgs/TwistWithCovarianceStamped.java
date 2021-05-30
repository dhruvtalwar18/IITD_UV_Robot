package geometry_msgs;

import org.ros.internal.message.Message;
import std_msgs.Header;

public interface TwistWithCovarianceStamped extends Message {
    public static final String _DEFINITION = "# This represents an estimated twist with reference coordinate frame and timestamp.\nHeader header\nTwistWithCovariance twist\n";
    public static final String _TYPE = "geometry_msgs/TwistWithCovarianceStamped";

    Header getHeader();

    TwistWithCovariance getTwist();

    void setHeader(Header header);

    void setTwist(TwistWithCovariance twistWithCovariance);
}
