package rosjava_test_msgs;

import org.ros.internal.message.Message;
import std_msgs.Header;

public interface TestHeader extends Message {
    public static final String _DEFINITION = "Header header\n\n# caller_id of most recent node to send this message\nstring caller_id\n# caller_id of the original node to send this message\nstring orig_caller_id\n\nbyte auto_header # autoset header on response\n";
    public static final String _TYPE = "rosjava_test_msgs/TestHeader";

    byte getAutoHeader();

    String getCallerId();

    Header getHeader();

    String getOrigCallerId();

    void setAutoHeader(byte b);

    void setCallerId(String str);

    void setHeader(Header header);

    void setOrigCallerId(String str);
}
