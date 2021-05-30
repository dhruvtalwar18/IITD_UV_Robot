package sensor_msgs;

import java.util.List;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.internal.message.Message;
import std_msgs.Header;

public interface PointCloud2 extends Message {
    public static final String _DEFINITION = "# This message holds a collection of N-dimensional points, which may\n# contain additional information such as normals, intensity, etc. The\n# point data is stored as a binary blob, its layout described by the\n# contents of the \"fields\" array.\n\n# The point cloud data may be organized 2d (image-like) or 1d\n# (unordered). Point clouds organized as 2d images may be produced by\n# camera depth sensors such as stereo or time-of-flight.\n\n# Time of sensor data acquisition, and the coordinate frame ID (for 3d\n# points).\nHeader header\n\n# 2D structure of the point cloud. If the cloud is unordered, height is\n# 1 and width is the length of the point cloud.\nuint32 height\nuint32 width\n\n# Describes the channels and their layout in the binary data blob.\nPointField[] fields\n\nbool    is_bigendian # Is this data bigendian?\nuint32  point_step   # Length of a point in bytes\nuint32  row_step     # Length of a row in bytes\nuint8[] data         # Actual point data, size is (row_step*height)\n\nbool is_dense        # True if there are no invalid points\n";
    public static final String _TYPE = "sensor_msgs/PointCloud2";

    ChannelBuffer getData();

    List<PointField> getFields();

    Header getHeader();

    int getHeight();

    boolean getIsBigendian();

    boolean getIsDense();

    int getPointStep();

    int getRowStep();

    int getWidth();

    void setData(ChannelBuffer channelBuffer);

    void setFields(List<PointField> list);

    void setHeader(Header header);

    void setHeight(int i);

    void setIsBigendian(boolean z);

    void setIsDense(boolean z);

    void setPointStep(int i);

    void setRowStep(int i);

    void setWidth(int i);
}
