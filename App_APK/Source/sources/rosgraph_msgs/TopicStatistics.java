package rosgraph_msgs;

import org.ros.internal.message.Message;
import org.ros.message.Duration;
import org.ros.message.Time;

public interface TopicStatistics extends Message {
    public static final String _DEFINITION = "# name of the topic\nstring topic\n\n# node id of the publisher\nstring node_pub\n\n# node id of the subscriber\nstring node_sub\n\n# the statistics apply to this time window\ntime window_start\ntime window_stop\n\n# number of messages delivered during the window\nint32 delivered_msgs\n# numbers of messages dropped during the window\nint32 dropped_msgs\n\n# traffic during the window, in bytes\nint32 traffic\n\n# mean/stddev/max period between two messages\nduration period_mean\nduration period_stddev\nduration period_max\n\n# mean/stddev/max age of the message based on the\n# timestamp in the message header. In case the\n# message does not have a header, it will be 0.\nduration stamp_age_mean\nduration stamp_age_stddev\nduration stamp_age_max\n";
    public static final String _TYPE = "rosgraph_msgs/TopicStatistics";

    int getDeliveredMsgs();

    int getDroppedMsgs();

    String getNodePub();

    String getNodeSub();

    Duration getPeriodMax();

    Duration getPeriodMean();

    Duration getPeriodStddev();

    Duration getStampAgeMax();

    Duration getStampAgeMean();

    Duration getStampAgeStddev();

    String getTopic();

    int getTraffic();

    Time getWindowStart();

    Time getWindowStop();

    void setDeliveredMsgs(int i);

    void setDroppedMsgs(int i);

    void setNodePub(String str);

    void setNodeSub(String str);

    void setPeriodMax(Duration duration);

    void setPeriodMean(Duration duration);

    void setPeriodStddev(Duration duration);

    void setStampAgeMax(Duration duration);

    void setStampAgeMean(Duration duration);

    void setStampAgeStddev(Duration duration);

    void setTopic(String str);

    void setTraffic(int i);

    void setWindowStart(Time time);

    void setWindowStop(Time time);
}
