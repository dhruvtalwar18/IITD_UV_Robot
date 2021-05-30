package geometry_msgs;

import org.ros.internal.message.Message;
import std_msgs.Header;

public interface AccelWithCovarianceStamped extends Message {
    public static final String _DEFINITION = "# This represents an estimated accel with reference coordinate frame and timestamp.\nHeader header\nAccelWithCovariance accel\n";
    public static final String _TYPE = "geometry_msgs/AccelWithCovarianceStamped";

    AccelWithCovariance getAccel();

    Header getHeader();

    void setAccel(AccelWithCovariance accelWithCovariance);

    void setHeader(Header header);
}
