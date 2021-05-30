package geometry_msgs;

import org.ros.internal.message.Message;

public interface Point32 extends Message {
    public static final String _DEFINITION = "# This contains the position of a point in free space(with 32 bits of precision).\n# It is recommeded to use Point wherever possible instead of Point32.  \n# \n# This recommendation is to promote interoperability.  \n#\n# This message is designed to take up less space when sending\n# lots of points at once, as in the case of a PointCloud.  \n\nfloat32 x\nfloat32 y\nfloat32 z";
    public static final String _TYPE = "geometry_msgs/Point32";

    float getX();

    float getY();

    float getZ();

    void setX(float f);

    void setY(float f);

    void setZ(float f);
}
