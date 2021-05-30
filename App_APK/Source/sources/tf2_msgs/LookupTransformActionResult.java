package tf2_msgs;

import actionlib_msgs.GoalStatus;
import org.ros.internal.message.Message;
import std_msgs.Header;

public interface LookupTransformActionResult extends Message {
    public static final String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\nHeader header\nactionlib_msgs/GoalStatus status\ntf2_msgs/LookupTransformResult result";
    public static final String _TYPE = "tf2_msgs/LookupTransformActionResult";

    Header getHeader();

    LookupTransformResult getResult();

    GoalStatus getStatus();

    void setHeader(Header header);

    void setResult(LookupTransformResult lookupTransformResult);

    void setStatus(GoalStatus goalStatus);
}