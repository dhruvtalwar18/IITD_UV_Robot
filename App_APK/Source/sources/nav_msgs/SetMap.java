package nav_msgs;

import org.ros.internal.message.Message;

public interface SetMap extends Message {
    public static final String _DEFINITION = "# Set a new map together with an initial pose\nnav_msgs/OccupancyGrid map\ngeometry_msgs/PoseWithCovarianceStamped initial_pose\n---\nbool success\n\n";
    public static final String _TYPE = "nav_msgs/SetMap";
}
