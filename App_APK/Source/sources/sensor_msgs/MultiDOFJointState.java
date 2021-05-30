package sensor_msgs;

import geometry_msgs.Transform;
import geometry_msgs.Twist;
import geometry_msgs.Wrench;
import java.util.List;
import org.ros.internal.message.Message;
import std_msgs.Header;

public interface MultiDOFJointState extends Message {
    public static final String _DEFINITION = "# Representation of state for joints with multiple degrees of freedom, \n# following the structure of JointState.\n#\n# It is assumed that a joint in a system corresponds to a transform that gets applied \n# along the kinematic chain. For example, a planar joint (as in URDF) is 3DOF (x, y, yaw)\n# and those 3DOF can be expressed as a transformation matrix, and that transformation\n# matrix can be converted back to (x, y, yaw)\n#\n# Each joint is uniquely identified by its name\n# The header specifies the time at which the joint states were recorded. All the joint states\n# in one message have to be recorded at the same time.\n#\n# This message consists of a multiple arrays, one for each part of the joint state. \n# The goal is to make each of the fields optional. When e.g. your joints have no\n# wrench associated with them, you can leave the wrench array empty. \n#\n# All arrays in this message should have the same size, or be empty.\n# This is the only way to uniquely associate the joint name with the correct\n# states.\n\nHeader header\n\nstring[] joint_names\ngeometry_msgs/Transform[] transforms\ngeometry_msgs/Twist[] twist\ngeometry_msgs/Wrench[] wrench\n";
    public static final String _TYPE = "sensor_msgs/MultiDOFJointState";

    Header getHeader();

    List<String> getJointNames();

    List<Transform> getTransforms();

    List<Twist> getTwist();

    List<Wrench> getWrench();

    void setHeader(Header header);

    void setJointNames(List<String> list);

    void setTransforms(List<Transform> list);

    void setTwist(List<Twist> list);

    void setWrench(List<Wrench> list);
}
