package sensor_msgs;

import org.ros.internal.message.Message;
import std_msgs.Header;

public interface RelativeHumidity extends Message {
    public static final String _DEFINITION = " # Single reading from a relative humidity sensor.  Defines the ratio of partial\n # pressure of water vapor to the saturated vapor pressure at a temperature.\n\n Header header             # timestamp of the measurement\n                           # frame_id is the location of the humidity sensor\n\n float64 relative_humidity # Expression of the relative humidity\n                           # from 0.0 to 1.0.\n                           # 0.0 is no partial pressure of water vapor\n                           # 1.0 represents partial pressure of saturation\n\n float64 variance          # 0 is interpreted as variance unknown";
    public static final String _TYPE = "sensor_msgs/RelativeHumidity";

    Header getHeader();

    double getRelativeHumidity();

    double getVariance();

    void setHeader(Header header);

    void setRelativeHumidity(double d);

    void setVariance(double d);
}
