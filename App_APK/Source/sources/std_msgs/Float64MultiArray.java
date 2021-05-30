package std_msgs;

import org.ros.internal.message.Message;

public interface Float64MultiArray extends Message {
    public static final String _DEFINITION = "# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nfloat64[]         data          # array of data\n\n";
    public static final String _TYPE = "std_msgs/Float64MultiArray";

    double[] getData();

    MultiArrayLayout getLayout();

    void setData(double[] dArr);

    void setLayout(MultiArrayLayout multiArrayLayout);
}
