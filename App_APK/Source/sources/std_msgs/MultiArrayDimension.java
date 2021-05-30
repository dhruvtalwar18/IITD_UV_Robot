package std_msgs;

import org.ros.internal.message.Message;

public interface MultiArrayDimension extends Message {
    public static final String _DEFINITION = "string label   # label of given dimension\nuint32 size    # size of given dimension (in type units)\nuint32 stride  # stride of given dimension";
    public static final String _TYPE = "std_msgs/MultiArrayDimension";

    String getLabel();

    int getSize();

    int getStride();

    void setLabel(String str);

    void setSize(int i);

    void setStride(int i);
}
