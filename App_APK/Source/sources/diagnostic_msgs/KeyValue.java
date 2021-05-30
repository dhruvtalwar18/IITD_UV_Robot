package diagnostic_msgs;

import org.ros.internal.message.Message;

public interface KeyValue extends Message {
    public static final String _DEFINITION = "string key # what to label this value when viewing\nstring value # a value to track over time\n";
    public static final String _TYPE = "diagnostic_msgs/KeyValue";

    String getKey();

    String getValue();

    void setKey(String str);

    void setValue(String str);
}
