package rocon_app_manager_msgs;

import org.ros.internal.message.Message;

public interface PublicInterface extends Message {
    public static final String _DEFINITION = "# Represents a public interface of a rapp\n\n# One of rocon_std_msgs.Connection type string constants ('publisher, 'subscriber', ...)\nstring connection_type\n\n# The data type, e.g. std_msgs/Strings\nstring data_type\n\n# Name of the topic/service/action xxx.\nstring name\n\n";
    public static final String _TYPE = "rocon_app_manager_msgs/PublicInterface";

    String getConnectionType();

    String getDataType();

    String getName();

    void setConnectionType(String str);

    void setDataType(String str);

    void setName(String str);
}
