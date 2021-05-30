package rocon_std_msgs;

import org.ros.internal.message.Message;

public interface KeyValue extends Message {
    public static final String _DEFINITION = "string key\nstring value";
    public static final String _TYPE = "rocon_std_msgs/KeyValue";

    String getKey();

    String getValue();

    void setKey(String str);

    void setValue(String str);
}
