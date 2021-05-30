package diagnostic_msgs;

import org.ros.internal.message.Message;

public interface SelfTest extends Message {
    public static final String _DEFINITION = "---\nstring id\nbyte passed\nDiagnosticStatus[] status\n";
    public static final String _TYPE = "diagnostic_msgs/SelfTest";
}
