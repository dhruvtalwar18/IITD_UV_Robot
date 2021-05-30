package sensor_msgs;

import org.ros.internal.message.Message;
import std_msgs.Header;

public interface FluidPressure extends Message {
    public static final String _DEFINITION = " # Single pressure reading.  This message is appropriate for measuring the\n # pressure inside of a fluid (air, water, etc).  This also includes\n # atmospheric or barometric pressure.\n\n # This message is not appropriate for force/pressure contact sensors.\n\n Header header           # timestamp of the measurement\n                         # frame_id is the location of the pressure sensor\n\n float64 fluid_pressure  # Absolute pressure reading in Pascals.\n\n float64 variance        # 0 is interpreted as variance unknown";
    public static final String _TYPE = "sensor_msgs/FluidPressure";

    double getFluidPressure();

    Header getHeader();

    double getVariance();

    void setFluidPressure(double d);

    void setHeader(Header header);

    void setVariance(double d);
}
