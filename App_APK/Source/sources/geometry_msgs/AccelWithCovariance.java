package geometry_msgs;

import org.ros.internal.message.Message;

public interface AccelWithCovariance extends Message {
    public static final String _DEFINITION = "# This expresses acceleration in free space with uncertainty.\n\nAccel accel\n\n# Row-major representation of the 6x6 covariance matrix\n# The orientation parameters use a fixed-axis representation.\n# In order, the parameters are:\n# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\nfloat64[36] covariance\n";
    public static final String _TYPE = "geometry_msgs/AccelWithCovariance";

    Accel getAccel();

    double[] getCovariance();

    void setAccel(Accel accel);

    void setCovariance(double[] dArr);
}
