package actionlib_msgs;

import org.ros.internal.message.Message;
import org.ros.message.Time;

public interface GoalID extends Message {
    public static final String _DEFINITION = "# The stamp should store the time at which this goal was requested.\n# It is used by an action server when it tries to preempt all\n# goals that were requested before a certain time\ntime stamp\n\n# The id provides a way to associate feedback and\n# result message with specific goal requests. The id\n# specified must be unique.\nstring id\n\n";
    public static final String _TYPE = "actionlib_msgs/GoalID";

    String getId();

    Time getStamp();

    void setId(String str);

    void setStamp(Time time);
}
