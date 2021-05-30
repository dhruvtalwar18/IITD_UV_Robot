package diagnostic_msgs;

import java.util.List;
import org.ros.internal.message.Message;
import std_msgs.Header;

public interface DiagnosticArray extends Message {
    public static final String _DEFINITION = "# This message is used to send diagnostic information about the state of the robot\nHeader header #for timestamp\nDiagnosticStatus[] status # an array of components being reported on";
    public static final String _TYPE = "diagnostic_msgs/DiagnosticArray";

    Header getHeader();

    List<DiagnosticStatus> getStatus();

    void setHeader(Header header);

    void setStatus(List<DiagnosticStatus> list);
}
