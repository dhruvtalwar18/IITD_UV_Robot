package std_msgs;

import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.internal.message.Message;

public interface ByteMultiArray extends Message {
    public static final String _DEFINITION = "# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nbyte[]            data          # array of data\n\n";
    public static final String _TYPE = "std_msgs/ByteMultiArray";

    ChannelBuffer getData();

    MultiArrayLayout getLayout();

    void setData(ChannelBuffer channelBuffer);

    void setLayout(MultiArrayLayout multiArrayLayout);
}
