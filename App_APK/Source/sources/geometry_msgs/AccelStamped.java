package geometry_msgs;

import org.ros.internal.message.Message;
import std_msgs.Header;

public interface AccelStamped extends Message {
    public static final String _DEFINITION = "# An accel with reference coordinate frame and timestamp\nHeader header\nAccel accel\n";
    public static final String _TYPE = "geometry_msgs/AccelStamped";

    Accel getAccel();

    Header getHeader();

    void setAccel(Accel accel);

    void setHeader(Header header);
}
