package nav_msgs;

import geometry_msgs.PoseWithCovariance;
import geometry_msgs.TwistWithCovariance;
import org.ros.internal.message.Message;
import std_msgs.Header;

public interface Odometry extends Message {
    public static final String _DEFINITION = "# This represents an estimate of a position and velocity in free space.  \n# The pose in this message should be specified in the coordinate frame given by header.frame_id.\n# The twist in this message should be specified in the coordinate frame given by the child_frame_id\nHeader header\nstring child_frame_id\ngeometry_msgs/PoseWithCovariance pose\ngeometry_msgs/TwistWithCovariance twist\n";
    public static final String _TYPE = "nav_msgs/Odometry";

    String getChildFrameId();

    Header getHeader();

    PoseWithCovariance getPose();

    TwistWithCovariance getTwist();

    void setChildFrameId(String str);

    void setHeader(Header header);

    void setPose(PoseWithCovariance poseWithCovariance);

    void setTwist(TwistWithCovariance twistWithCovariance);
}
