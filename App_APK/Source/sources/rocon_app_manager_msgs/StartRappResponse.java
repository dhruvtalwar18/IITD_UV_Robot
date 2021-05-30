package rocon_app_manager_msgs;

import org.ros.internal.message.Message;

public interface StartRappResponse extends Message {
    public static final String _DEFINITION = "bool started\n\n# classifying start success/failure, see ErrorCodes.msg\nint32 error_code\n\n# human friendly string for debugging (usually upon error)\nstring message\n\n# Namespace where the rapp interface can be found\nstring application_namespace";
    public static final String _TYPE = "rocon_app_manager_msgs/StartRappResponse";

    String getApplicationNamespace();

    int getErrorCode();

    String getMessage();

    boolean getStarted();

    void setApplicationNamespace(String str);

    void setErrorCode(int i);

    void setMessage(String str);

    void setStarted(boolean z);
}
