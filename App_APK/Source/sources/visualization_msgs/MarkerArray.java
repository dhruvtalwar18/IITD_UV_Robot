package visualization_msgs;

import java.util.List;
import org.ros.internal.message.Message;

public interface MarkerArray extends Message {
    public static final String _DEFINITION = "Marker[] markers\n";
    public static final String _TYPE = "visualization_msgs/MarkerArray";

    List<Marker> getMarkers();

    void setMarkers(List<Marker> list);
}
