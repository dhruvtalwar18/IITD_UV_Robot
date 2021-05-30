package actionlib_msgs;

import java.util.List;
import org.ros.internal.message.Message;
import std_msgs.Header;

public interface GoalStatusArray extends Message {
    public static final String _DEFINITION = "# Stores the statuses for goals that are currently being tracked\n# by an action server\nHeader header\nGoalStatus[] status_list\n\n";
    public static final String _TYPE = "actionlib_msgs/GoalStatusArray";

    Header getHeader();

    List<GoalStatus> getStatusList();

    void setHeader(Header header);

    void setStatusList(List<GoalStatus> list);
}
