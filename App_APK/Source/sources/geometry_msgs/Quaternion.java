package geometry_msgs;

import org.ros.internal.message.Message;

public interface Quaternion extends Message {
    public static final String _DEFINITION = "# This represents an orientation in free space in quaternion form.\n\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 w\n";
    public static final String _TYPE = "geometry_msgs/Quaternion";

    double getW();

    double getX();

    double getY();

    double getZ();

    void setW(double d);

    void setX(double d);

    void setY(double d);

    void setZ(double d);
}
