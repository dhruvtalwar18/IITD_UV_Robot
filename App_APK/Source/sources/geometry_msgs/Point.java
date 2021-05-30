package geometry_msgs;

import org.ros.internal.message.Message;

public interface Point extends Message {
    public static final String _DEFINITION = "# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n";
    public static final String _TYPE = "geometry_msgs/Point";

    double getX();

    double getY();

    double getZ();

    void setX(double d);

    void setY(double d);

    void setZ(double d);
}
