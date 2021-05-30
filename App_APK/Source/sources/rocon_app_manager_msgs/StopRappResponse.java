package rocon_app_manager_msgs;

import org.ros.internal.message.Message;

public interface StopRappResponse extends Message {
    public static final String _DEFINITION = "# true if app stopped, false otherwise\nbool stopped\n# classifying start success/failure, see ErrorCodes.msg\nint32 error_code\n# human friendly string for debugging (usually upon error)\nstring message";
    public static final String _TYPE = "rocon_app_manager_msgs/StopRappResponse";

    int getErrorCode();

    String getMessage();

    boolean getStopped();

    void setErrorCode(int i);

    void setMessage(String str);

    void setStopped(boolean z);
}
