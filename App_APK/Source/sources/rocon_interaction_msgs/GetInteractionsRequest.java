package rocon_interaction_msgs;

import java.util.List;
import org.ros.internal.message.Message;

public interface GetInteractionsRequest extends Message {
    public static final String _DEFINITION = "# For remocons who need to retrieve a filtered list of interactions\n# appropriate to the role and platform they will run for/on. \n\n# Filter according to roles. If not specified it will return \n# interactions for all roles.\nstring[] roles\n\n# Filter according to the rocon uri of the requesting device. T\n# If not specified, the manager will assume 'rocon://' (wildcards)\nstring uri\n";
    public static final String _TYPE = "rocon_interaction_msgs/GetInteractionsRequest";

    List<String> getRoles();

    String getUri();

    void setRoles(List<String> list);

    void setUri(String str);
}
