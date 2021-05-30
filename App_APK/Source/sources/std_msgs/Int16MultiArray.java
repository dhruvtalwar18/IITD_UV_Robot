package std_msgs;

import org.ros.internal.message.Message;

public interface Int16MultiArray extends Message {
    public static final String _DEFINITION = "# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nint16[]           data          # array of data\n\n";
    public static final String _TYPE = "std_msgs/Int16MultiArray";

    short[] getData();

    MultiArrayLayout getLayout();

    void setData(short[] sArr);

    void setLayout(MultiArrayLayout multiArrayLayout);
}
