package nav_msgs;

import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.internal.message.Message;
import std_msgs.Header;

public interface OccupancyGrid extends Message {
    public static final String _DEFINITION = "# This represents a 2-D grid map, in which each cell represents the probability of\n# occupancy.\n\nHeader header \n\n#MetaData for the map\nMapMetaData info\n\n# The map data, in row-major order, starting with (0,0).  Occupancy\n# probabilities are in the range [0,100].  Unknown is -1.\nint8[] data\n";
    public static final String _TYPE = "nav_msgs/OccupancyGrid";

    ChannelBuffer getData();

    Header getHeader();

    MapMetaData getInfo();

    void setData(ChannelBuffer channelBuffer);

    void setHeader(Header header);

    void setInfo(MapMetaData mapMetaData);
}
