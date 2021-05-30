package nav_msgs;

import org.ros.internal.message.Message;

public interface GetMapResponse extends Message {
    public static final String _DEFINITION = "nav_msgs/OccupancyGrid map";
    public static final String _TYPE = "nav_msgs/GetMapResponse";

    OccupancyGrid getMap();

    void setMap(OccupancyGrid occupancyGrid);
}
