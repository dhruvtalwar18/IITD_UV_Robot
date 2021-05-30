package nav_msgs;

import org.ros.internal.message.Message;

public interface GetPlanResponse extends Message {
    public static final String _DEFINITION = "nav_msgs/Path plan";
    public static final String _TYPE = "nav_msgs/GetPlanResponse";

    Path getPlan();

    void setPlan(Path path);
}
