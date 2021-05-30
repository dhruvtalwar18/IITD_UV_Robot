package sensor_msgs;

import geometry_msgs.Quaternion;
import geometry_msgs.Vector3;
import org.ros.internal.message.Message;
import std_msgs.Header;

public interface Imu extends Message {
    public static final String _DEFINITION = "# This is a message to hold data from an IMU (Inertial Measurement Unit)\n#\n# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec\n#\n# If the covariance of the measurement is known, it should be filled in (if all you know is the \n# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)\n# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the\n# data a covariance will have to be assumed or gotten from some other source\n#\n# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation \n# estimate), please set element 0 of the associated covariance matrix to -1\n# If you are interpreting this message, please check for a value of -1 in the first element of each \n# covariance matrix, and disregard the associated estimate.\n\nHeader header\n\ngeometry_msgs/Quaternion orientation\nfloat64[9] orientation_covariance # Row major about x, y, z axes\n\ngeometry_msgs/Vector3 angular_velocity\nfloat64[9] angular_velocity_covariance # Row major about x, y, z axes\n\ngeometry_msgs/Vector3 linear_acceleration\nfloat64[9] linear_acceleration_covariance # Row major x, y z \n";
    public static final String _TYPE = "sensor_msgs/Imu";

    Vector3 getAngularVelocity();

    double[] getAngularVelocityCovariance();

    Header getHeader();

    Vector3 getLinearAcceleration();

    double[] getLinearAccelerationCovariance();

    Quaternion getOrientation();

    double[] getOrientationCovariance();

    void setAngularVelocity(Vector3 vector3);

    void setAngularVelocityCovariance(double[] dArr);

    void setHeader(Header header);

    void setLinearAcceleration(Vector3 vector3);

    void setLinearAccelerationCovariance(double[] dArr);

    void setOrientation(Quaternion quaternion);

    void setOrientationCovariance(double[] dArr);
}
