package rocon_app_manager_msgs;

import org.ros.internal.message.Message;

public interface PublishedInterface extends Message {
    public static final String _DEFINITION = "# The runtime, possibly namespaced and remapped public interface.\n\nPublicInterface interface\n\n# Final absolute namespaced/remapped name of the topic/service/action xxx\nstring name";
    public static final String _TYPE = "rocon_app_manager_msgs/PublishedInterface";

    PublicInterface getInterface();

    String getName();

    void setInterface(PublicInterface publicInterface);

    void setName(String str);
}
