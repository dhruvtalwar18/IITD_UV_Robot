package sensor_msgs;

import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.internal.message.Message;
import std_msgs.Header;

public interface Image extends Message {
    public static final String _DEFINITION = "# This message contains an uncompressed image\n# (0, 0) is at top-left corner of image\n#\n\nHeader header        # Header timestamp should be acquisition time of image\n                     # Header frame_id should be optical frame of camera\n                     # origin of frame should be optical center of camera\n                     # +x should point to the right in the image\n                     # +y should point down in the image\n                     # +z should point into to plane of the image\n                     # If the frame_id here and the frame_id of the CameraInfo\n                     # message associated with the image conflict\n                     # the behavior is undefined\n\nuint32 height         # image height, that is, number of rows\nuint32 width          # image width, that is, number of columns\n\n# The legal values for encoding are in file src/image_encodings.cpp\n# If you want to standardize a new string format, join\n# ros-users@lists.sourceforge.net and send an email proposing a new encoding.\n\nstring encoding       # Encoding of pixels -- channel meaning, ordering, size\n                      # taken from the list of strings in include/sensor_msgs/image_encodings.h\n\nuint8 is_bigendian    # is this data bigendian?\nuint32 step           # Full row length in bytes\nuint8[] data          # actual matrix data, size is (step * rows)\n";
    public static final String _TYPE = "sensor_msgs/Image";

    ChannelBuffer getData();

    String getEncoding();

    Header getHeader();

    int getHeight();

    byte getIsBigendian();

    int getStep();

    int getWidth();

    void setData(ChannelBuffer channelBuffer);

    void setEncoding(String str);

    void setHeader(Header header);

    void setHeight(int i);

    void setIsBigendian(byte b);

    void setStep(int i);

    void setWidth(int i);
}
