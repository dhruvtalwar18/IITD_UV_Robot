package diagnostic_msgs;

import java.util.List;
import org.ros.internal.message.Message;

public interface SelfTestResponse extends Message {
    public static final String _DEFINITION = "string id\nbyte passed\nDiagnosticStatus[] status";
    public static final String _TYPE = "diagnostic_msgs/SelfTestResponse";

    String getId();

    byte getPassed();

    List<DiagnosticStatus> getStatus();

    void setId(String str);

    void setPassed(byte b);

    void setStatus(List<DiagnosticStatus> list);
}
