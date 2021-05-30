package std_msgs;

import org.ros.internal.message.Message;
import org.ros.message.Time;

public interface Header extends Message {
    public static final String _DEFINITION = "# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data \n# in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing ID \nuint32 seq\n#Two-integer timestamp that is expressed as:\n# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n# time-handling sugar is provided by the client library\ntime stamp\n#Frame this data is associated with\n# 0: no frame\n# 1: global frame\nstring frame_id\n";
    public static final String _TYPE = "std_msgs/Header";

    String getFrameId();

    int getSeq();

    Time getStamp();

    void setFrameId(String str);

    void setSeq(int i);

    void setStamp(Time time);
}
