package sensor_msgs;

import java.util.List;
import org.ros.internal.message.Message;

public interface JoyFeedbackArray extends Message {
    public static final String _DEFINITION = "# This message publishes values for multiple feedback at once. \nJoyFeedback[] array";
    public static final String _TYPE = "sensor_msgs/JoyFeedbackArray";

    List<JoyFeedback> getArray();

    void setArray(List<JoyFeedback> list);
}
