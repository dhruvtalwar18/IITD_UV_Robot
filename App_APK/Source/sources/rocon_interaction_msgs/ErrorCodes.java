package rocon_interaction_msgs;

import org.ros.internal.message.Message;

public interface ErrorCodes extends Message {
    public static final byte ALREADY_PAIRING = 32;
    public static final byte INTERACTION_QUOTA_REACHED = 21;
    public static final byte INTERACTION_UNAVAILABLE = 20;
    public static final String MSG_ALREADY_PAIRING = "\"Already pairing, cannot start another pairing.\"";
    public static final String MSG_INTERACTION_QUOTA_REACHED = "\"More connections of this type not permitted.\"";
    public static final String MSG_INTERACTION_UNAVAILABLE = "\"This role-app pair is not available for use.\"";
    public static final String MSG_START_PAIRED_RAPP_FAILED = "\"Failed to start the paired rapp.\"";
    public static final byte START_PAIRED_RAPP_FAILED = 31;
    public static final byte SUCCESS = 0;
    public static final String _DEFINITION = "# Error types for interactions\n\nint8 SUCCESS = 0\n\n# Interaction errors\nint8 INTERACTION_UNAVAILABLE = 20    # The requested role-app pair is not available\nint8 INTERACTION_QUOTA_REACHED = 21  # Maximum number or role-app connections already reached \n\n# Pairing errors\nint8 START_PAIRED_RAPP_FAILED = 31   # If a paired rapp fails to start.\nint8 ALREADY_PAIRING          = 32   # If trying to start a pairing, but already pairing\n\nstring MSG_INTERACTION_UNAVAILABLE   = \"This role-app pair is not available for use.\"\nstring MSG_INTERACTION_QUOTA_REACHED = \"More connections of this type not permitted.\"\nstring MSG_START_PAIRED_RAPP_FAILED  = \"Failed to start the paired rapp.\"\nstring MSG_ALREADY_PAIRING           = \"Already pairing, cannot start another pairing.\"";
    public static final String _TYPE = "rocon_interaction_msgs/ErrorCodes";
}
