package rosjava_test_msgs;

import org.ros.internal.message.Message;

public interface AddTwoIntsResponse extends Message {
    public static final String _DEFINITION = "int64 sum";
    public static final String _TYPE = "rosjava_test_msgs/AddTwoIntsResponse";

    long getSum();

    void setSum(long j);
}
