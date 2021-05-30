package sensor_msgs;

import org.ros.internal.message.Message;
import org.ros.message.Time;
import std_msgs.Header;

public interface TimeReference extends Message {
    public static final String _DEFINITION = "# Measurement from an external time source not actively synchronized with the system clock.\n\nHeader header    # stamp is system time for which measurement was valid\n                 # frame_id is not used \n\ntime   time_ref  # corresponding time from this external source\nstring source    # (optional) name of time source\n";
    public static final String _TYPE = "sensor_msgs/TimeReference";

    Header getHeader();

    String getSource();

    Time getTimeRef();

    void setHeader(Header header);

    void setSource(String str);

    void setTimeRef(Time time);
}
