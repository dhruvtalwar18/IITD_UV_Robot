package nav_msgs;

import org.ros.internal.message.Message;

public interface SetMapResponse extends Message {
    public static final String _DEFINITION = "bool success";
    public static final String _TYPE = "nav_msgs/SetMapResponse";

    boolean getSuccess();

    void setSuccess(boolean z);
}
