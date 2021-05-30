package diagnostic_msgs;

import org.ros.internal.message.Message;

public interface AddDiagnosticsResponse extends Message {
    public static final String _DEFINITION = "\n# True if diagnostic aggregator was updated with new diagnostics, False\n# otherwise. A false return value means that either there is a bond in the\n# aggregator which already used the requested namespace, or the initialization\n# of analyzers failed.\nbool success\n\n# Message with additional information about the success or failure\nstring message";
    public static final String _TYPE = "diagnostic_msgs/AddDiagnosticsResponse";

    String getMessage();

    boolean getSuccess();

    void setMessage(String str);

    void setSuccess(boolean z);
}
