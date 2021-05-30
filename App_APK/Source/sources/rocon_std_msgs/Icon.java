package rocon_std_msgs;

import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.internal.message.Message;

public interface Icon extends Message {
    public static final String _DEFINITION = "# Used to idenfity the original package/filename resource this icon was/is to be loaded from\n# This typically doesn't have to be set, but can be very useful when loading icons from yaml definitions.\nstring resource_name\n\n# Image data format.  \"jpeg\" or \"png\"\nstring format\n\n# Image data.\nuint8[] data";
    public static final String _TYPE = "rocon_std_msgs/Icon";

    ChannelBuffer getData();

    String getFormat();

    String getResourceName();

    void setData(ChannelBuffer channelBuffer);

    void setFormat(String str);

    void setResourceName(String str);
}
