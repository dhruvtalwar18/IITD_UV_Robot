package sensor_msgs;

import geometry_msgs.Vector3;
import org.ros.internal.message.Message;
import std_msgs.Header;

public interface MagneticField extends Message {
    public static final String _DEFINITION = " # Measurement of the Magnetic Field vector at a specific location.\n\n # If the covariance of the measurement is known, it should be filled in\n # (if all you know is the variance of each measurement, e.g. from the datasheet,\n #just put those along the diagonal)\n # A covariance matrix of all zeros will be interpreted as \"covariance unknown\",\n # and to use the data a covariance will have to be assumed or gotten from some\n # other source\n\n\n Header header                        # timestamp is the time the\n                                      # field was measured\n                                      # frame_id is the location and orientation\n                                      # of the field measurement\n\n geometry_msgs/Vector3 magnetic_field # x, y, and z components of the\n                                      # field vector in Tesla\n                                      # If your sensor does not output 3 axes,\n                                      # put NaNs in the components not reported.\n\n float64[9] magnetic_field_covariance # Row major about x, y, z axes\n                                      # 0 is interpreted as variance unknown";
    public static final String _TYPE = "sensor_msgs/MagneticField";

    Header getHeader();

    Vector3 getMagneticField();

    double[] getMagneticFieldCovariance();

    void setHeader(Header header);

    void setMagneticField(Vector3 vector3);

    void setMagneticFieldCovariance(double[] dArr);
}
