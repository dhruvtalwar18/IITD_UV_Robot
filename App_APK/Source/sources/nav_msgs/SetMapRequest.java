package nav_msgs;

import geometry_msgs.PoseWithCovarianceStamped;
import org.ros.internal.message.Message;

public interface SetMapRequest extends Message {
    public static final String _DEFINITION = "# Set a new map together with an initial pose\nnav_msgs/OccupancyGrid map\ngeometry_msgs/PoseWithCovarianceStamped initial_pose\n";
    public static final String _TYPE = "nav_msgs/SetMapRequest";

    PoseWithCovarianceStamped getInitialPose();

    OccupancyGrid getMap();

    void setInitialPose(PoseWithCovarianceStamped poseWithCovarianceStamped);

    void setMap(OccupancyGrid occupancyGrid);
}
