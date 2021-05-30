package geometry_msgs;

import java.util.List;
import org.ros.internal.message.Message;

public interface Polygon extends Message {
    public static final String _DEFINITION = "#A specification of a polygon where the first and last points are assumed to be connected\nPoint32[] points\n";
    public static final String _TYPE = "geometry_msgs/Polygon";

    List<Point32> getPoints();

    void setPoints(List<Point32> list);
}
