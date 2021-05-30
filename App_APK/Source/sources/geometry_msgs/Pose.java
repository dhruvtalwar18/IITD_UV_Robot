package geometry_msgs;

import org.ros.internal.message.Message;

public interface Pose extends Message {
    public static final String _DEFINITION = "# A representation of pose in free space, composed of position and orientation. \nPoint position\nQuaternion orientation\n";
    public static final String _TYPE = "geometry_msgs/Pose";

    Quaternion getOrientation();

    Point getPosition();

    void setOrientation(Quaternion quaternion);

    void setPosition(Point point);
}
