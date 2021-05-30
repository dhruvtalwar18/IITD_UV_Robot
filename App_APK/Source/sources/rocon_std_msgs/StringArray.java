package rocon_std_msgs;

import java.util.List;
import org.ros.internal.message.Message;

public interface StringArray extends Message {
    public static final String _DEFINITION = "string[] strings\n\n";
    public static final String _TYPE = "rocon_std_msgs/StringArray";

    List<String> getStrings();

    void setStrings(List<String> list);
}
