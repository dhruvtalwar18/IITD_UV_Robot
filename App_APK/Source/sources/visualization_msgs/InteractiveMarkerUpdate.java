package visualization_msgs;

import java.util.List;
import org.ros.internal.message.Message;

public interface InteractiveMarkerUpdate extends Message {
    public static final byte KEEP_ALIVE = 0;
    public static final byte UPDATE = 1;
    public static final String _DEFINITION = "# Identifying string. Must be unique in the topic namespace\n# that this server works on.\nstring server_id\n\n# Sequence number.\n# The client will use this to detect if it has missed an update.\nuint64 seq_num\n\n# Type holds the purpose of this message.  It must be one of UPDATE or KEEP_ALIVE.\n# UPDATE: Incremental update to previous state. \n#         The sequence number must be 1 higher than for\n#         the previous update.\n# KEEP_ALIVE: Indicates the that the server is still living.\n#             The sequence number does not increase.\n#             No payload data should be filled out (markers, poses, or erases).\nuint8 KEEP_ALIVE = 0\nuint8 UPDATE = 1\n\nuint8 type\n\n#Note: No guarantees on the order of processing.\n#      Contents must be kept consistent by sender.\n\n#Markers to be added or updated\nInteractiveMarker[] markers\n\n#Poses of markers that should be moved\nInteractiveMarkerPose[] poses\n\n#Names of markers to be erased\nstring[] erases\n";
    public static final String _TYPE = "visualization_msgs/InteractiveMarkerUpdate";

    List<String> getErases();

    List<InteractiveMarker> getMarkers();

    List<InteractiveMarkerPose> getPoses();

    long getSeqNum();

    String getServerId();

    byte getType();

    void setErases(List<String> list);

    void setMarkers(List<InteractiveMarker> list);

    void setPoses(List<InteractiveMarkerPose> list);

    void setSeqNum(long j);

    void setServerId(String str);

    void setType(byte b);
}
