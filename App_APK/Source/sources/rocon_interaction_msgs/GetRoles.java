package rocon_interaction_msgs;

import org.ros.internal.message.Message;

public interface GetRoles extends Message {
    public static final String _DEFINITION = "# For remocons who need to retrieve a filtered list of roles\n# appropriate to the role and platform they will run for/on. \n\n# Filter according to the rocon uri of the requesting device. T\n# If not specified, the manager will assume 'rocon://' (wildcards)\nstring uri\n---\nstring[] roles\n";
    public static final String _TYPE = "rocon_interaction_msgs/GetRoles";
}
