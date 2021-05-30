package sensor_msgs;

import geometry_msgs.Point32;
import java.util.List;
import org.ros.internal.message.Message;
import std_msgs.Header;

public interface PointCloud extends Message {
    public static final String _DEFINITION = "# This message holds a collection of 3d points, plus optional additional\n# information about each point.\n\n# Time of sensor data acquisition, coordinate frame ID.\nHeader header\n\n# Array of 3d points. Each Point32 should be interpreted as a 3d point\n# in the frame given in the header.\ngeometry_msgs/Point32[] points\n\n# Each channel should have the same number of elements as points array,\n# and the data in each channel should correspond 1:1 with each point.\n# Channel names in common practice are listed in ChannelFloat32.msg.\nChannelFloat32[] channels\n";
    public static final String _TYPE = "sensor_msgs/PointCloud";

    List<ChannelFloat32> getChannels();

    Header getHeader();

    List<Point32> getPoints();

    void setChannels(List<ChannelFloat32> list);

    void setHeader(Header header);

    void setPoints(List<Point32> list);
}
