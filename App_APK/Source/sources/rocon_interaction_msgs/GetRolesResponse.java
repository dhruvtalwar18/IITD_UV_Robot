package rocon_interaction_msgs;

import java.util.List;
import org.ros.internal.message.Message;

public interface GetRolesResponse extends Message {
    public static final String _DEFINITION = "string[] roles";
    public static final String _TYPE = "rocon_interaction_msgs/GetRolesResponse";

    List<String> getRoles();

    void setRoles(List<String> list);
}
