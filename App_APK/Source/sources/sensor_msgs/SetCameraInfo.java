package sensor_msgs;

import org.ros.internal.message.Message;

public interface SetCameraInfo extends Message {
    public static final String _DEFINITION = "# This service requests that a camera stores the given CameraInfo \n# as that camera's calibration information.\n#\n# The width and height in the camera_info field should match what the\n# camera is currently outputting on its camera_info topic, and the camera\n# will assume that the region of the imager that is being referred to is\n# the region that the camera is currently capturing.\n\nsensor_msgs/CameraInfo camera_info # The camera_info to store\n---\nbool success          # True if the call succeeded\nstring status_message # Used to give details about success\n";
    public static final String _TYPE = "sensor_msgs/SetCameraInfo";
}
