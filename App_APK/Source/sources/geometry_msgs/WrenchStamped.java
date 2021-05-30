package geometry_msgs;

import org.ros.internal.message.Message;
import std_msgs.Header;

public interface WrenchStamped extends Message {
    public static final String _DEFINITION = "# A wrench with reference coordinate frame and timestamp\nHeader header\nWrench wrench\n";
    public static final String _TYPE = "geometry_msgs/WrenchStamped";

    Header getHeader();

    Wrench getWrench();

    void setHeader(Header header);

    void setWrench(Wrench wrench);
}
