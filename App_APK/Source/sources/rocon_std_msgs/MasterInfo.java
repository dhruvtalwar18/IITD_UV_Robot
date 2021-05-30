package rocon_std_msgs;

import org.ros.internal.message.Message;

public interface MasterInfo extends Message {
    public static final String _DEFINITION = "# Publish basic information about the concert.\n#\n# Fixed variables\n#   version\n# Over-ridable variables:\n#   name\n#   description\n#   icon\n# Runtime variables:\n#   id\n#\n\nstring name\n#uuid_msgs/UniqueID id\nstring description\nrocon_std_msgs/Icon icon\nstring version";
    public static final String _TYPE = "rocon_std_msgs/MasterInfo";

    String getDescription();

    Icon getIcon();

    String getName();

    String getVersion();

    void setDescription(String str);

    void setIcon(Icon icon);

    void setName(String str);

    void setVersion(String str);
}
