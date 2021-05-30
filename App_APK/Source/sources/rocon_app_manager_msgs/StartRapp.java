package rocon_app_manager_msgs;

import org.ros.internal.message.Message;

public interface StartRapp extends Message {
    public static final String _DEFINITION = "# Name of the rapp to launch\nstring name\nrocon_std_msgs/Remapping[] remappings\n\n# Key value pairs representing rapp parameters\nrocon_std_msgs/KeyValue[] parameters\n---\nbool started\n\n# classifying start success/failure, see ErrorCodes.msg\nint32 error_code\n\n# human friendly string for debugging (usually upon error)\nstring message\n\n# Namespace where the rapp interface can be found\nstring application_namespace\n";
    public static final String _TYPE = "rocon_app_manager_msgs/StartRapp";
}
