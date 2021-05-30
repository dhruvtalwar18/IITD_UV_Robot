package rosjava_test_msgs;

import org.ros.internal.message.Message;

public interface CompositeB extends Message {
    public static final String _DEFINITION = "# copy of geometry_msgs/Point for testing\nfloat64 x\nfloat64 y\nfloat64 z\n";
    public static final String _TYPE = "rosjava_test_msgs/CompositeB";

    double getX();

    double getY();

    double getZ();

    void setX(double d);

    void setY(double d);

    void setZ(double d);
}
