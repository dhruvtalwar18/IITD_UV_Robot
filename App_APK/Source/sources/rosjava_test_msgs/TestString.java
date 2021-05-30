package rosjava_test_msgs;

import org.ros.internal.message.Message;

public interface TestString extends Message {
    public static final String _DEFINITION = "# Integration test message\n# caller_id of most recent node to send this message\nstring caller_id\n# caller_id of the original node to send this message\nstring orig_caller_id\nstring data\n";
    public static final String _TYPE = "rosjava_test_msgs/TestString";

    String getCallerId();

    String getData();

    String getOrigCallerId();

    void setCallerId(String str);

    void setData(String str);

    void setOrigCallerId(String str);
}
