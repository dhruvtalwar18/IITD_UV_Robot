package rocon_std_msgs;

import org.ros.internal.message.Message;

public interface Connection extends Message {
    public static final String ACTION_CLIENT = "action_client";
    public static final String ACTION_SERVER = "action_server";
    public static final String INVALID = "invalid";
    public static final String PUBLISHER = "publisher";
    public static final String SERVICE = "service";
    public static final String SUBSCRIBER = "subscriber";
    public static final String _DEFINITION = "# A connection can be 1 of 5 types\nstring PUBLISHER = publisher\nstring SUBSCRIBER = subscriber\nstring SERVICE = service\nstring ACTION_CLIENT = action_client\nstring ACTION_SERVER = action_server\nstring INVALID = invalid\n\n# type of connection (from string constants above)\nstring type\n\n# this is the topic/service name or the action base name\nstring name \n\n# the name of the node establishing this connection\nstring node\n\n# topic, service or action type, e.g. std_msgs/String\nstring type_info\n\n# xmlrpc node uri for managing the connection\nstring xmlrpc_uri";
    public static final String _TYPE = "rocon_std_msgs/Connection";

    String getName();

    String getNode();

    String getType();

    String getTypeInfo();

    String getXmlrpcUri();

    void setName(String str);

    void setNode(String str);

    void setType(String str);

    void setTypeInfo(String str);

    void setXmlrpcUri(String str);
}
