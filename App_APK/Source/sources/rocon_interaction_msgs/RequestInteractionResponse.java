package rocon_interaction_msgs;

import org.ros.internal.message.Message;

public interface RequestInteractionResponse extends Message {
    public static final String _DEFINITION = "\nbool result\n\n# classifying start success/failure, see ErrorCodes.msg\nint8 error_code\n\n# human friendly string for debugging (usually upon error)\nstring message";
    public static final String _TYPE = "rocon_interaction_msgs/RequestInteractionResponse";

    byte getErrorCode();

    String getMessage();

    boolean getResult();

    void setErrorCode(byte b);

    void setMessage(String str);

    void setResult(boolean z);
}
