package std_msgs;

import org.ros.internal.message.Message;

public interface UInt64MultiArray extends Message {
    public static final String _DEFINITION = "# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nuint64[]          data          # array of data\n\n";
    public static final String _TYPE = "std_msgs/UInt64MultiArray";

    long[] getData();

    MultiArrayLayout getLayout();

    void setData(long[] jArr);

    void setLayout(MultiArrayLayout multiArrayLayout);
}
