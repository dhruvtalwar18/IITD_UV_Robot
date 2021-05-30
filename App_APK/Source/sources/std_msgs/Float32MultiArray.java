package std_msgs;

import org.ros.internal.message.Message;

public interface Float32MultiArray extends Message {
    public static final String _DEFINITION = "# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nfloat32[]         data          # array of data\n\n";
    public static final String _TYPE = "std_msgs/Float32MultiArray";

    float[] getData();

    MultiArrayLayout getLayout();

    void setData(float[] fArr);

    void setLayout(MultiArrayLayout multiArrayLayout);
}
