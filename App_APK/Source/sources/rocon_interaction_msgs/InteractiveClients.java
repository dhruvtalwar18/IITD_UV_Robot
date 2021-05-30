package rocon_interaction_msgs;

import java.util.List;
import org.ros.internal.message.Message;

public interface InteractiveClients extends Message {
    public static final String _DEFINITION = "# \n# Two lists are included here to distinguish between idle clients and those that\n# are currently running an app.\n#\nInteractiveClient[] idle_clients\nInteractiveClient[] running_clients\n";
    public static final String _TYPE = "rocon_interaction_msgs/InteractiveClients";

    List<InteractiveClient> getIdleClients();

    List<InteractiveClient> getRunningClients();

    void setIdleClients(List<InteractiveClient> list);

    void setRunningClients(List<InteractiveClient> list);
}
