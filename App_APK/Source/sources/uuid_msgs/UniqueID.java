package uuid_msgs;

import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.internal.message.Message;

public interface UniqueID extends Message {
    public static final String _DEFINITION = "# A universally unique identifier (UUID).\n#\n#  http://en.wikipedia.org/wiki/Universally_unique_identifier\n#  http://tools.ietf.org/html/rfc4122.html\n\nuint8[16] uuid\n";
    public static final String _TYPE = "uuid_msgs/UniqueID";

    ChannelBuffer getUuid();

    void setUuid(ChannelBuffer channelBuffer);
}
