package rocon_app_manager_msgs;

import org.ros.internal.message.Message;

public interface ErrorCodes extends Message {
    public static final byte ALREADY_REMOTE_CONTROLLED = 32;
    public static final byte CLIENT_CONNECTION_DISRUPTED = 36;
    public static final byte INVITING_CONTROLLER_BLACKLISTED = 35;
    public static final byte INVITING_CONTROLLER_NOT_WHITELISTED = 34;
    public static final byte LOCAL_INVITATIONS_ONLY = 30;
    public static final byte MULTI_RAPP_NOT_SUPPORTED = 10;
    public static final byte NOT_CURRENT_REMOTE_CONTROLLER = 33;
    public static final byte NO_LOCAL_GATEWAY = 31;
    public static final byte RAPP_IS_NOT_AVAILABLE = 21;
    public static final byte RAPP_IS_NOT_RUNNING = 20;
    public static final byte SUCCESS = 0;
    public static final byte UNKNOWN = 1;
    public static final String _DEFINITION = "# Error types for the rocon app manager\n\n# General\nint8 SUCCESS = 0\nint8 UNKNOWN = 1\n\n# Start\nint8 MULTI_RAPP_NOT_SUPPORTED = 10\n\n# Stop Rapp\nint8 RAPP_IS_NOT_RUNNING = 20\nint8 RAPP_IS_NOT_AVAILABLE = 21\n\n# Invitations\nint8 LOCAL_INVITATIONS_ONLY = 30\nint8 NO_LOCAL_GATEWAY = 31\nint8 ALREADY_REMOTE_CONTROLLED = 32\nint8 NOT_CURRENT_REMOTE_CONTROLLER = 33\nint8 INVITING_CONTROLLER_NOT_WHITELISTED = 34\nint8 INVITING_CONTROLLER_BLACKLISTED = 35\nint8 CLIENT_CONNECTION_DISRUPTED = 36";
    public static final String _TYPE = "rocon_app_manager_msgs/ErrorCodes";
}
