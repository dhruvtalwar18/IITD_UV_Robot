package geometry_msgs;

import org.ros.internal.message.Message;

public interface Pose2D extends Message {
    public static final String _DEFINITION = "# Deprecated\n# Please use the full 3D pose.\n\n# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.\n\n# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.\n\n\n# This expresses a position and orientation on a 2D manifold.\n\nfloat64 x\nfloat64 y\nfloat64 theta\n";
    public static final String _TYPE = "geometry_msgs/Pose2D";

    double getTheta();

    double getX();

    double getY();

    void setTheta(double d);

    void setX(double d);

    void setY(double d);
}
