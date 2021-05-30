package std_msgs;

import org.ros.internal.message.Message;

public interface Int32MultiArray extends Message {
    public static final String _DEFINITION = "# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nint32[]           data          # array of data\n\n";
    public static final String _TYPE = "std_msgs/Int32MultiArray";

    int[] getData();

    MultiArrayLayout getLayout();

    void setData(int[] iArr);

    void setLayout(MultiArrayLayout multiArrayLayout);
}
