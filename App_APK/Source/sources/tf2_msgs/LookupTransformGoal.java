package tf2_msgs;

import org.ros.internal.message.Message;
import org.ros.message.Duration;
import org.ros.message.Time;

public interface LookupTransformGoal extends Message {
    public static final String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n#goal definition#Simple API\nstring target_frame\nstring source_frame\ntime source_time\nduration timeout\n\n#Advanced API\ntime target_time\nstring fixed_frame\n\n#Whether or not to use the advanced API\nbool advanced\n\n";
    public static final String _TYPE = "tf2_msgs/LookupTransformGoal";

    boolean getAdvanced();

    String getFixedFrame();

    String getSourceFrame();

    Time getSourceTime();

    String getTargetFrame();

    Time getTargetTime();

    Duration getTimeout();

    void setAdvanced(boolean z);

    void setFixedFrame(String str);

    void setSourceFrame(String str);

    void setSourceTime(Time time);

    void setTargetFrame(String str);

    void setTargetTime(Time time);

    void setTimeout(Duration duration);
}
