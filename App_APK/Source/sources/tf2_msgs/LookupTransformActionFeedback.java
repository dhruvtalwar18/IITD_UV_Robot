package tf2_msgs;

import actionlib_msgs.GoalStatus;
import org.ros.internal.message.Message;
import std_msgs.Header;

public interface LookupTransformActionFeedback extends Message {
    public static final String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\nHeader header\nactionlib_msgs/GoalStatus status\ntf2_msgs/LookupTransformFeedback feedback";
    public static final String _TYPE = "tf2_msgs/LookupTransformActionFeedback";

    LookupTransformFeedback getFeedback();

    Header getHeader();

    GoalStatus getStatus();

    void setFeedback(LookupTransformFeedback lookupTransformFeedback);

    void setHeader(Header header);

    void setStatus(GoalStatus goalStatus);
}
