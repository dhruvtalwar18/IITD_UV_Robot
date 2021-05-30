package rocon_app_manager_msgs;

import org.ros.internal.message.Message;

public interface InviteResponse extends Message {
    public static final String _DEFINITION = "bool result\n# classifying start success/failure, see ErrorCodes.msg (to be implemented)\nint32 error_code\n# human friendly string for debugging (usually upon error)\nstring message";
    public static final String _TYPE = "rocon_app_manager_msgs/InviteResponse";

    int getErrorCode();

    String getMessage();

    boolean getResult();

    void setErrorCode(int i);

    void setMessage(String str);

    void setResult(boolean z);
}
