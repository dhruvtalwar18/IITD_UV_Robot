package geometry_msgs;

import org.ros.internal.message.Message;
import std_msgs.Header;

public interface PolygonStamped extends Message {
    public static final String _DEFINITION = "# This represents a Polygon with reference coordinate frame and timestamp\nHeader header\nPolygon polygon\n";
    public static final String _TYPE = "geometry_msgs/PolygonStamped";

    Header getHeader();

    Polygon getPolygon();

    void setHeader(Header header);

    void setPolygon(Polygon polygon);
}
