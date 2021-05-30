package sensor_msgs;

import org.ros.internal.message.Message;
import std_msgs.Header;

public interface Temperature extends Message {
    public static final String _DEFINITION = " # Single temperature reading.\n\n Header header           # timestamp is the time the temperature was measured\n                         # frame_id is the location of the temperature reading\n\n float64 temperature     # Measurement of the Temperature in Degrees Celsius\n\n float64 variance        # 0 is interpreted as variance unknown";
    public static final String _TYPE = "sensor_msgs/Temperature";

    Header getHeader();

    double getTemperature();

    double getVariance();

    void setHeader(Header header);

    void setTemperature(double d);

    void setVariance(double d);
}
