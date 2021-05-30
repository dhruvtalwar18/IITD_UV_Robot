package geometry_msgs;

import org.ros.internal.message.Message;
import std_msgs.Header;

public interface PointStamped extends Message {
    public static final String _DEFINITION = "# This represents a Point with reference coordinate frame and timestamp\nHeader header\nPoint point\n";
    public static final String _TYPE = "geometry_msgs/PointStamped";

    Header getHeader();

    Point getPoint();

    void setHeader(Header header);

    void setPoint(Point point);
}
