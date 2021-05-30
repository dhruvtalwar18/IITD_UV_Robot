package rocon_service_pair_msgs;

import org.ros.internal.message.Message;

public interface TestiesRequest extends Message {
    public static final String _DEFINITION = "string data\n";
    public static final String _TYPE = "rocon_service_pair_msgs/TestiesRequest";

    String getData();

    void setData(String str);
}
