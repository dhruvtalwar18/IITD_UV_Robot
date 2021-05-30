package nav_msgs;

import geometry_msgs.PoseStamped;
import java.util.List;
import org.ros.internal.message.Message;
import std_msgs.Header;

public interface Path extends Message {
    public static final String _DEFINITION = "#An array of poses that represents a Path for a robot to follow\nHeader header\ngeometry_msgs/PoseStamped[] poses\n";
    public static final String _TYPE = "nav_msgs/Path";

    Header getHeader();

    List<PoseStamped> getPoses();

    void setHeader(Header header);

    void setPoses(List<PoseStamped> list);
}
