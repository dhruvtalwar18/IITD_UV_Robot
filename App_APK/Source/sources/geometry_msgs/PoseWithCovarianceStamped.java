package geometry_msgs;

import org.ros.internal.message.Message;
import std_msgs.Header;

public interface PoseWithCovarianceStamped extends Message {
    public static final String _DEFINITION = "# This expresses an estimated pose with a reference coordinate frame and timestamp\n\nHeader header\nPoseWithCovariance pose\n";
    public static final String _TYPE = "geometry_msgs/PoseWithCovarianceStamped";

    Header getHeader();

    PoseWithCovariance getPose();

    void setHeader(Header header);

    void setPose(PoseWithCovariance poseWithCovariance);
}
