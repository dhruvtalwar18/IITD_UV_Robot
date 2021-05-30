package rocon_app_manager_msgs;

import org.ros.internal.message.Message;

public interface StopRapp extends Message {
    public static final String _DEFINITION = "---\n# true if app stopped, false otherwise\nbool stopped\n# classifying start success/failure, see ErrorCodes.msg\nint32 error_code\n# human friendly string for debugging (usually upon error)\nstring message\n\n";
    public static final String _TYPE = "rocon_app_manager_msgs/StopRapp";
}
