package rocon_interaction_msgs;

import java.util.List;
import org.ros.internal.message.Message;
import rocon_std_msgs.PlatformInfo;
import uuid_msgs.UniqueID;

public interface InteractiveClient extends Message {
    public static final String _DEFINITION = "# \n# Describes an interactive concert client.\n#\n# Unique names - human consumable rocon name as well as globally unique name\n# provided by the remocon client.\n\nstring name\nuuid_msgs/UniqueID id\n\nrocon_std_msgs/PlatformInfo platform_info\n\n# easy to read display names of interactions running on this remocon\nstring[] running_interactions\n";
    public static final String _TYPE = "rocon_interaction_msgs/InteractiveClient";

    UniqueID getId();

    String getName();

    PlatformInfo getPlatformInfo();

    List<String> getRunningInteractions();

    void setId(UniqueID uniqueID);

    void setName(String str);

    void setPlatformInfo(PlatformInfo platformInfo);

    void setRunningInteractions(List<String> list);
}
