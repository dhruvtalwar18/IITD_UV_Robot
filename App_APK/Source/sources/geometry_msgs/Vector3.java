package geometry_msgs;

import org.ros.internal.message.Message;

public interface Vector3 extends Message {
    public static final String _DEFINITION = "# This represents a vector in free space. \n# It is only meant to represent a direction. Therefore, it does not\n# make sense to apply a translation to it (e.g., when applying a \n# generic rigid transformation to a Vector3, tf2 will only apply the\n# rotation). If you want your data to be translatable too, use the\n# geometry_msgs/Point message instead.\n\nfloat64 x\nfloat64 y\nfloat64 z";
    public static final String _TYPE = "geometry_msgs/Vector3";

    double getX();

    double getY();

    double getZ();

    void setX(double d);

    void setY(double d);

    void setZ(double d);
}
