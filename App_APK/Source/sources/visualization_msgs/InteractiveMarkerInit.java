package visualization_msgs;

import java.util.List;
import org.ros.internal.message.Message;

public interface InteractiveMarkerInit extends Message {
    public static final String _DEFINITION = "# Identifying string. Must be unique in the topic namespace\n# that this server works on.\nstring server_id\n\n# Sequence number.\n# The client will use this to detect if it has missed a subsequent\n# update.  Every update message will have the same sequence number as\n# an init message.  Clients will likely want to unsubscribe from the\n# init topic after a successful initialization to avoid receiving\n# duplicate data.\nuint64 seq_num\n\n# All markers.\nInteractiveMarker[] markers\n";
    public static final String _TYPE = "visualization_msgs/InteractiveMarkerInit";

    List<InteractiveMarker> getMarkers();

    long getSeqNum();

    String getServerId();

    void setMarkers(List<InteractiveMarker> list);

    void setSeqNum(long j);

    void setServerId(String str);
}
