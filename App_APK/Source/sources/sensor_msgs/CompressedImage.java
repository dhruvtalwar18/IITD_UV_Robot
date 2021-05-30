package sensor_msgs;

import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.internal.message.Message;
import std_msgs.Header;

public interface CompressedImage extends Message {
    public static final String _DEFINITION = "# This message contains a compressed image\n\nHeader header        # Header timestamp should be acquisition time of image\n                     # Header frame_id should be optical frame of camera\n                     # origin of frame should be optical center of camera\n                     # +x should point to the right in the image\n                     # +y should point down in the image\n                     # +z should point into to plane of the image\n\nstring format        # Specifies the format of the data\n                     #   Acceptable values:\n                     #     jpeg, png\nuint8[] data         # Compressed image buffer\n";
    public static final String _TYPE = "sensor_msgs/CompressedImage";

    ChannelBuffer getData();

    String getFormat();

    Header getHeader();

    void setData(ChannelBuffer channelBuffer);

    void setFormat(String str);

    void setHeader(Header header);
}
