package sensor_msgs;

import org.ros.internal.message.Message;

public interface SetCameraInfoResponse extends Message {
    public static final String _DEFINITION = "bool success          # True if the call succeeded\nstring status_message # Used to give details about success";
    public static final String _TYPE = "sensor_msgs/SetCameraInfoResponse";

    String getStatusMessage();

    boolean getSuccess();

    void setStatusMessage(String str);

    void setSuccess(boolean z);
}
